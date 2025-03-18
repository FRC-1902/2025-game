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
import edu.wpi.first.math.util.Units;
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
import frc.robot.commands.ContinuousConditionalCommand;
import frc.robot.commands.ElevatorFactory;
import frc.robot.commands.algaeIntake.AlgaeIntakeCommand;
import frc.robot.commands.algaeIntake.AlgaeOuttakeCommand;
import frc.robot.commands.endEffector.EndEffectorFactory;
import frc.robot.commands.endEffector.ScoreCommand;
import frc.robot.commands.drive.ObjectAlign;
import frc.robot.commands.drive.SnapToWaypoint;
import frc.robot.commands.floorIntake.AutoIntakeFactory;
import frc.robot.commands.floorIntake.PositionIntakeCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.DetectionSubsystem;

/*
 * Publishes a network table chooser to smart dashboard to select the autonomous command. 
 * Compose the auto routines here to put in the selector.
 */
public class AutoSelector {
  private LoggedDashboardChooser<Command> autoChooser;
  private String pushingP = "Auto/EnablePushingP";

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
    autoIntakeFactory = new AutoIntakeFactory(floorIntake, elevator, endEffector, led);
    elevatorFactory = new ElevatorFactory(endEffector, elevator, floorIntake);


    autoChooser = new LoggedDashboardChooser<>("Auto/Auto Chooser");
    
    SmartDashboard.putBoolean(pushingP, false);
    SmartDashboard.setPersistent(pushingP);

    autoChooser.addDefaultOption("Do Nothing", getDoNothingAuto());
    autoChooser.addDefaultOption("Leave", getLeave());
   // autoChooser.addOption("Right 4 L3", getRight4L3());
   // autoChooser.addOption("Left 4 L3", getLeft4L3());
    autoChooser.addOption("1 L2", get1L2());
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
      new ObjectAlign(detectionSubsystem, swerve, floorIntake)
    );
  } 

  private Command getPushingPCommand() {
    return new ParallelRaceGroup(
      new WaitCommand(2),
      new SnapToWaypoint(swerve, () -> {
        // Get current pose
        Pose2d currentPose = swerve.getPose();
        
        double distance = Units.inchesToMeters(-4); 
        
        Translation2d backwardsVector = new Translation2d(
          distance * Math.cos(currentPose.getRotation().getRadians()), 
          distance * Math.sin(currentPose.getRotation().getRadians())
        );
        
        Translation2d targetTranslation = currentPose.getTranslation().plus(backwardsVector);
        
        return new Pose2d(targetTranslation, currentPose.getRotation());
      })
    );
  } 

  /**
   * Checks if the PushingP SmartDashboard toggle is enabled
   */
  private boolean isPushingPEnabled() {
    return SmartDashboard.getBoolean(pushingP, false);
  }

  // Auto definitions

  /**
   * Auto that doesn't do anything
   */
  private Command getDoNothingAuto() {
    return setStartPosition(7.140, 7.500);
  }

  /**
   * Auto that leaves the starting line
   */
  private Command getLeave() {
    return new SequentialCommandGroup(
      setStartPosition(7.140, 7.500),
      new ConditionalCommand(
        getPushingPCommand(),
        new InstantCommand(),
        this::isPushingPEnabled
      ),
      swerve.getFollowPathCommand("Leave")
    );
  }

  /**
   * Auto that leaves the starting line
   */
  private Command get1L2() {
    return new SequentialCommandGroup(
      setStartPosition(7.182, 4.182),

      new ConditionalCommand(
        getPushingPCommand(),
        new InstantCommand(),
        this::isPushingPEnabled
      ),

      endEffectorFactory.getIndexSequence(),

      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L2),
        swerve.getFollowPathCommand("1 L2 1"),
        new AlgaeIntakeCommand(algaeIntake)
      ),
      // Place
      new ScoreCommand(endEffector),
      swerve.getFollowPathCommand("1 L2 2")

    );
  }

  private SequentialCommandGroup getRight4L3(){
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(7.14, 2.240), // TODO: fix start pos

      new ConditionalCommand(
        getPushingPCommand(),
        new InstantCommand(),
        this::isPushingPEnabled
      ),

      endEffectorFactory.getIndexSequence(),

      // Drive to reef and grab algae
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getFollowPathCommand("4 L3 1"),
        new AlgaeIntakeCommand(algaeIntake)
      ),
      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE ONE

      // Drive to HP

      // drive far enough away to get rid of algae

      // drive, while outaking algae, intaking, and looking for a piece

      new ParallelCommandGroup(
        swerve.getFollowPathCommand("4 L3 2"),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new AlgaeOuttakeCommand(algaeIntake),
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
        )
      ),
      getCoralWithDetection(),

      // Drive to reef
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getPathfindToPathCommand("4 L3 3")
      ),
      
      // Place 
      new ScoreCommand(endEffector),

      // END OF CYCLE TWO

      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN),
        swerve.getFollowPathCommand("4 L3 4")
      ),
      getCoralWithDetection(),

      // Drive to reef & grab algae
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getPathfindToPathCommand("4 L3 5"),
        new AlgaeIntakeCommand(algaeIntake)
      ),
      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE THREE

      // Drive to get coral
      new ParallelCommandGroup(
        swerve.getFollowPathCommand("4 L3 6"),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new AlgaeOuttakeCommand(algaeIntake),
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
        )
      ),
      
      getCoralWithDetection(),

      // Drive to reef
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getPathfindToPathCommand("4 L3 7")
      ),

      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE FOUR
      elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)

      // End of Auto
    );
  }

  private SequentialCommandGroup getLeft4L3(){
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(7.14, 5.8118),

      // Pushing P
      new ConditionalCommand(
        getPushingPCommand(),
        new InstantCommand(),
        this::isPushingPEnabled
      ),

      endEffectorFactory.getIndexSequence(),

      // Drive to reef and grab algae
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getFollowPathCommand("4 L3 1 Left"),
        new AlgaeIntakeCommand(algaeIntake)
      ),
      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE ONE

      // Drive to HP

      // drive far enough away to get rid of algae

      // drive, while outaking algae, intaking, and looking for a piece

      new ParallelCommandGroup(
        swerve.getFollowPathCommand("4 L3 2 Left"),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new AlgaeOuttakeCommand(algaeIntake),
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
        )
      ),
      getCoralWithDetection(),

      // Drive to reef
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getPathfindToPathCommand("4 L3 3 Left")
      ),
      
      // Place 
      new ScoreCommand(endEffector),

      // END OF CYCLE TWO

      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN),
        swerve.getFollowPathCommand("4 L3 4 Left")
      ),
      getCoralWithDetection(),

      // Drive to reef & grab algae
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getPathfindToPathCommand("4 L3 5 Left"),
        new AlgaeIntakeCommand(algaeIntake)
      ),
      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE THREE

      // Drive to get coral
      new ParallelCommandGroup(
        swerve.getFollowPathCommand("4 L3 6 Left"),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new AlgaeOuttakeCommand(algaeIntake),
          elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
        )
      ),
      
      getCoralWithDetection(),

      // Drive to reef
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getPathfindToPathCommand("4 L3 7 Left")
      ),

      // Place
      new ScoreCommand(endEffector),

      // END OF CYCLE FOUR
      elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)

      // End of Auto
    );
  }
}

