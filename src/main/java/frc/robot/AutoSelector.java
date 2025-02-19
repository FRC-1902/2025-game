package frc.robot;

import java.lang.reflect.Field;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoPlaceFactory;
import frc.robot.commands.ContinuousConditionalCommand;
import frc.robot.commands.ElevatorFactory;
import frc.robot.commands.PlaceCommand;
import frc.robot.commands.drive.ObjectAlign;
import frc.robot.commands.intake.DeployFloorIntakeCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.DetectionSubsystem;
import frc.robot.FieldConstants;
import frc.robot.Constants.EndEffector;

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

  DeployFloorIntakeCommand deployFloorIntakeCommand;
  AutoPlaceFactory autoPlaceFactory;
  ElevatorFactory elevatorFactory;

  ContinuousConditionalCommand continuousConditionalCommand;
  
  public AutoSelector(RobotContainer robotContainer, ContinuousConditionalCommand continuousConditionalCommand) {
    swerve = robotContainer.swerve;
    algaeIntake = robotContainer.algaeIntake;
    floorIntake = robotContainer.floorIntake;
    endEffector = robotContainer.endEffector;
    detectionSubsystem = robotContainer.detectionSubsystem;
    this.continuousConditionalCommand = continuousConditionalCommand;

    autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    autoChooser.addDefaultOption("Do Nothing", getDoNothingAuto());
    autoChooser.addOption("Test 5 Piece", getTest5());

    SmartDashboard.putData("Auto Choices", autoChooser.getSendableChooser());
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
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d(x, y, Rotation2d.fromDegrees(180)))), // TODO: Check if correct
        new InstantCommand(() -> swerve.zeroGyroWithAlliance())
      ),
      new SequentialCommandGroup(
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d(FieldConstants.LENGTH - x, FieldConstants.WIDTH - y, Rotation2d.fromDegrees(0)))),  // TODO: Check if correct
        new InstantCommand(() -> swerve.zeroGyroWithAlliance())
      ),
      this::isBlue
    );
  }

  private Command getPieceRoutine(String PathName){
    return new ContinuousConditionalCommand(
      swerve.getFollowPathCommand(PathName),
      new ObjectAlign(detectionSubsystem, swerve),
      () -> detectionSubsystem.isTargetVisible()
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
  private SequentialCommandGroup getTest5() {
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(20, 10),
      // drive to reef
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getFollowPathCommand("1.path")
      ),
      new PlaceCommand(endEffector),
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN),
        swerve.getFollowPathCommand("1.2.path")
      )
    );
  }

  //
  private SequentialCommandGroup getSpecialAuto(){
    return new SequentialCommandGroup(
      // setup odometry
      setStartPosition(20, 10),
      // drive to reef
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.L3),
        swerve.getFollowPathCommand("1.path")
      ),
      new PlaceCommand(endEffector),
      new ParallelCommandGroup(
        elevatorFactory.getElevatorCommand(Constants.Elevator.Position.MIN),
        getPieceRoutine("gussy is special")
      )
    );
  }
}