package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LED;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeOuttakeCommand;
import frc.robot.commands.AutoPlaceFactory;
import frc.robot.commands.ClimbFactory;
import frc.robot.commands.ElevatorFactory;
import frc.robot.commands.EndEffectorFactory;
import frc.robot.commands.PlaceCommand;
import frc.robot.commands.drive.AutoDriveFactory;
import frc.robot.commands.intake.AutoIntakeFactory;
import frc.robot.commands.intake.DeployFloorIntakeCommand;
import frc.robot.commands.intake.OuttakeFloorIntakeCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

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
  ElevatorSubsystem elevator;

  DeployFloorIntakeCommand deployFloorIntakeCommand;
  ElevatorFactory elevatorFactory;
  AutoIntakeFactory autoIntakeFactory;
  EndEffectorFactory endEffectorFactory;
  
  public AutoSelector(RobotContainer robotContainer) {
    // this.robotContainer = robotContainer;
    swerve = robotContainer.swerve;
    algaeIntake = robotContainer.algaeIntake;
    floorIntake = robotContainer.floorIntake;
    endEffector = robotContainer.endEffector;
    elevator = robotContainer.elevator;

    endEffectorFactory = new EndEffectorFactory(endEffector);
    autoIntakeFactory = new AutoIntakeFactory(floorIntake, elevator, endEffector, endEffectorFactory);
    elevatorFactory = new ElevatorFactory(endEffector, elevator, floorIntake);

    autoChooser = new LoggedDashboardChooser<>("Auto/Auto Chooser");

    autoChooser.addDefaultOption("Do Nothing", getDoNothingAuto());
    autoChooser.addOption("Test 5 Piece", sqaure222());
    autoChooser.addOption("3 L3", get3L3());
    autoChooser.addOption("Sqaure", sqaure());
    autoChooser.addOption("3 L3 Test", get3L3Test());
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

  private Command setStartPosition(double x, double y) {
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

  // Auto definitions

  /**
   * Auto that doesn't do anything
   */
  private Command getDoNothingAuto() {
    return setStartPosition(0, 0); // TODO: Configure
  }

  /**
   * Test Auto Top, not complete, for example
   */
  private SequentialCommandGroup sqaure() {
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(2.850, 4.000),
      swerve.getFollowPathCommand("s1"),
      swerve.getFollowPathCommand("s2"),
      swerve.getFollowPathCommand("s3"),
      swerve.getFollowPathCommand("s4")
    );
  }

  private SequentialCommandGroup sqaure222() {
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(20, 10),
      // drive to reef
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getFollowPathCommand("1")
      ),
      new PlaceCommand(endEffector),
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN),
        swerve.getFollowPathCommand("1.2")
      )
    );
  }

  /**
   * 3 L3 From blue right
   */
  private SequentialCommandGroup get3L3() {
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
      new PlaceCommand(endEffector),
      // Drive to HP
      new ParallelCommandGroup(
        swerve.getFollowPathCommand("3 L3 2"),
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
      ),
      // Intake
      //new AlgaeOuttakeCommand(algaeIntake),
      autoIntakeFactory.getIntakeSequence(Constants.FloorIntake.HP_ANGLE),
      
      new WaitCommand(3),
      // Drive to reef
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getFollowPathCommand("3 L3 3")
      ),
      // Place 
      new PlaceCommand(endEffector),
      new ParallelCommandGroup(
        swerve.getFollowPathCommand("3 L3 4"),
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN)
      ),
      autoIntakeFactory.getIntakeSequence(Constants.FloorIntake.HP_ANGLE),
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getFollowPathCommand("3 L3 5")
      ),
      new PlaceCommand(endEffector)
    );
  }
  private SequentialCommandGroup get3L3Test() {
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(7.170, 2.200),
      swerve.getFollowPathCommand("3 L3 1"),
      swerve.getFollowPathCommand("3 L3 2"),
      swerve.getFollowPathCommand("3 L3 3"), 
      swerve.getFollowPathCommand("3 L3 4"),
      swerve.getFollowPathCommand("3 L3 5")
    );
  }
}