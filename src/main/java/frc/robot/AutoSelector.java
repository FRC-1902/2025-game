package frc.robot;

import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LED;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeOuttakeCommand;
import frc.robot.commands.ContinuousConditionalCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorFactory;
import frc.robot.commands.drive.AutoDriveFactory;
import frc.robot.commands.endEffector.EndEffectorFactory;
import frc.robot.commands.endEffector.ScoreCommand;
import frc.robot.commands.drive.ObjectAlign;
import frc.robot.commands.drive.PathToFollowPath;
import frc.robot.commands.floorIntake.AutoIntakeFactory;
import frc.robot.commands.floorIntake.OuttakeCommand;
import frc.robot.commands.floorIntake.PositionIntakeCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.DetectionSubsystem;
import frc.robot.FieldConstants;
import frc.robot.Constants.EndEffector;
import frc.robot.commands.endEffector.ScoreCommand;
import frc.robot.commands.drive.DriveToObject;
import frc.robot.Constants;

/*
 * Publishes a network table chooser to smart dashboard to select the autonomous command. 
 * Compose the auto routines here to put in the selector.
 */
public class AutoSelector {
  private LoggedDashboardChooser<Command> autoChooser;

  SwerveSubsystem swerve;
  AlgaeIntakeSubsystem algaeIntake;
  FloorIntakeSubsystem floorIntake;
  EndEffectorSubsystem endEffector;
  DetectionSubsystem detectionSubsystem;
  ElevatorSubsystem elevator;

  PositionIntakeCommand deployFloorIntakeCommand;
  AutoIntakeFactory autoIntakeFactory;
  ElevatorFactory elevatorFactory;
  ObjectAlign objectAlign;
  PathToFollowPath pathToFollowPath;

  ContinuousConditionalCommand continuousConditionalCommand;
  EndEffectorFactory endEffectorFactory;
  LEDSubsystem led;
  
  public AutoSelector(RobotContainer robotContainer) {
    // this.robotContainer = robotContainer;
    swerve = robotContainer.swerve;
    elevator = robotContainer.elevator;
    algaeIntake = robotContainer.algaeIntake;
    floorIntake = robotContainer.floorIntake;
    endEffector = robotContainer.endEffector;
    detectionSubsystem = robotContainer.detectionSubsystem;
    elevator = robotContainer.elevator;
    led = robotContainer.led;

    endEffectorFactory = new EndEffectorFactory(endEffector);
    autoIntakeFactory = new AutoIntakeFactory(floorIntake, elevator, endEffector, endEffectorFactory, led);
    elevatorFactory = new ElevatorFactory(endEffector, elevator, floorIntake);

    autoChooser = new LoggedDashboardChooser<>("Auto/Auto Chooser");

    autoChooser.addDefaultOption("Do Nothing", getDoNothingAuto());
    autoChooser.addOption("3 L3 Test", get3L3Test());
    autoChooser.addOption("3 L3", getTheEverythingApp());
  }

  /**
   * @return The selected auto from smart dashboard
  */
  public Command getSelectedCommand() {
    return autoChooser.get();
  }

  private boolean isBlue() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Blue;
    } else {
      return false; // true for default to blue alliance
    }
  }

  public Command setStartPosition(double x, double y) {
    return new ConditionalCommand(
      new SequentialCommandGroup(
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d(x, y, Rotation2d.fromDegrees(180)))),
        new InstantCommand(() -> swerve.zeroGyroWithAlliance())
      ),
      new SequentialCommandGroup(
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d(FieldConstants.LENGTH - x, FieldConstants.WIDTH - y, Rotation2d.fromDegrees(0)))), 
        new InstantCommand(() -> swerve.zeroGyroWithAlliance())
      ),
      this::isBlue
    );
  }

  /**
   * Aligns and drives to coral and ends once intaken
   * <b><h1>Gussy likes to call this "get verified"</h1></b>
   * @param pathName
   * @return
   */
  private Command getCoralWithDetection(){
    return new ParallelDeadlineGroup(
      new ParallelRaceGroup(
        new WaitCommand(7),
        autoIntakeFactory.getAutonomousIntakeSequence(Constants.FloorIntake.FLOOR_ANGLE)
      ),
      
      // follow path, breaking out if you see a piece and drive to that piece instead
      // new ContinuousConditionalCommand(
      //   swerve.getFollowPathCommand(pathName),
      //   new SequentialCommandGroup(
      //     new ObjectAlign(detectionSubsystem, swerve),
      //     new DriveToObject(swerve, detectionSubsystem)
      //   ),
      //   () -> detectionSubsystem.isTargetVisible()
      // )
      new ConditionalCommand(
        new SequentialCommandGroup(
          new ObjectAlign(detectionSubsystem, swerve, floorIntake)
          // new DriveToObject(swerve, floorIntake)
        ),
        new InstantCommand(),
        () -> detectionSubsystem.isTargetVisible()
      )
    );
  }
  // private Command getCoralWithDetection(String pathName){
  //   return new ParallelRaceGroup(
  //     autoIntakeFactory.getAutonomousIntakeSequence(Constants.FloorIntake.FLOOR_ANGLE),
  //     // follow path, breaking out if you see a piece and drive to that piece instead
  //     new ContinuousConditionalCommand(
  //       swerve.getFollowPathCommand(pathName),
  //       new SequentialCommandGroup(
  //         new ObjectAlign(detectionSubsystem, swerve),
  //         new DriveToObject(swerve, floorIntake)
  //       ),
  //       () -> detectionSubsystem.isTargetVisible()
  //     )
  //   );
  // }
  

  // Auto definitions

  /**
   * Auto that doesn't do anything
   */
  private Command getDoNothingAuto() {
    return setStartPosition(0, 0); // TODO: Configure
  }

  private SequentialCommandGroup get3L3Test() {
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(7.170, 2.200),
      swerve.getFollowPathCommand("3 L3 1"),
      swerve.getFollowPathCommand("3 L3 2a"),
      swerve.getFollowPathCommand("3 L3 2b"),
      swerve.getFollowPathCommand("3 L3 3"), 
      swerve.getFollowPathCommand("3 L3 4"),
      swerve.getFollowPathCommand("3 L3 5")
    );
  }

  private SequentialCommandGroup getTheEverythingApp(){
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(7.170, 2.200), // TODO: fix start pos

      endEffectorFactory.getIndexSequence(),

      // Drive to reef and grab algae
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getFollowPathCommand("3 L3 1"),
        new AlgaeIntakeCommand(algaeIntake)
      ),
      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE ONE

      // Drive to HP

      // drive far enough away to get rid of algae

      // swerve.getFollowPathCommand("3 L3 2"),
      // drive, while outaking algae, intaking, and looking for a piece
      swerve.getFollowPathCommand("3 L3 2a"),

      new ParallelCommandGroup(
        new AlgaeOuttakeCommand(algaeIntake),
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
      ),
      getCoralWithDetection(),

      // Drive to reef
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        new PathToFollowPath("3 L3 3", swerve)
      ),
      
      // Place 
      new ScoreCommand(endEffector),

      // END OF CYCLE TWO

      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN),
        swerve.getFollowPathCommand("3 L3 4")
      ),
      getCoralWithDetection(),

      // Drive to reef & grab algae
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        new PathToFollowPath("3 L3 5", swerve),
        new AlgaeIntakeCommand(algaeIntake)
      ),
      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE THREE

      // TODO: gracefully put stuff down
      new SequentialCommandGroup(
        swerve.getFollowPathCommand("3 L3 6"),
        new ElevatorCommand(elevator, Constants.Elevator.Position.MIN)
      )
    );
  }

  private SequentialCommandGroup getBluesky(){
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(7.170, 2.200),
      // drive and suck
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getFollowPathCommand("3 L3 1")
        //new AlgaeIntakeCommand(algaeIntake)
      ),
      // Place
      new ScoreCommand(endEffector),
      // Drive to HP
      new ParallelCommandGroup(
        swerve.getFollowPathCommand("some path that is essentially fencesitting"),
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
      ),
      // Intake
      //new AlgaeOuttakeCommand(algaeIntake),
      new ObjectAlign(detectionSubsystem, swerve, floorIntake), // allign
      autoIntakeFactory.getIntakeSequence(Constants.FloorIntake.FLOOR_ANGLE),
      
      new WaitCommand(3),
      // Drive to reef
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getFollowPathCommand("3 L3 3")
      ),
      // Place 
      new ScoreCommand(endEffector),
      new ParallelCommandGroup(
        swerve.getFollowPathCommand("Some path that makes the comments angry"),
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
      ),
      new ObjectAlign(detectionSubsystem, swerve, floorIntake),
      autoIntakeFactory.getIntakeSequence(Constants.FloorIntake.FLOOR_ANGLE),
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getFollowPathCommand("3 L3 5")
      ),
      new ScoreCommand(endEffector)
    );
  }
}
