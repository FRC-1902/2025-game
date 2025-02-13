package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathPlannerPath;

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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AlgaeIntake;
import frc.robot.commands.AutoPlaceFactory;
import frc.robot.commands.drive.AutoDriveFactory;
import frc.robot.commands.intake.DeployFloorIntakeCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/*
 * Publishes a network table chooser to smart dashboard to select the autonomous command. 
 * Compose the auto routines here to put in the selector.
 */
public class AutoSelector {
  private LoggedDashboardChooser<Command> autoChooser;
  private LoggedDashboardChooser<String> alternativeSelector;

  RobotContainer robotContainer;
  SwerveSubsystem swerve;
  AlgaeIntakeSubsystem algaeIntake;
  FloorIntakeSubsystem floorIntake;
  EndEffectorSubsystem endEffector;

  DeployFloorIntakeCommand deployFloorIntakeCommand;
  AutoPlaceFactory AutoPlaceFactory;
  
  public AutoSelector(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    swerve = robotContainer.swerve;
    algaeIntake = robotContainer.algaeIntake;
    floorIntake = robotContainer.floorIntake;
    endEffector = robotContainer.endEffector;

    autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    autoChooser.addDefaultOption("Do Nothing", getDoNothingAuto());
    autoChooser.addOption("Test 5 Piece", getTest5());

    SmartDashboard.putData("Auto Choices", autoChooser.getSendableChooser());
  }

  /**
   * @return The selected auto from smart dashboard
  */
  public Command getSelectedCommand() {
    DataLogManager.log("Sending command: " + autoChooser.get().toString());
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

  // Auto definitions

  private SequentialCommandGroup getDoNothingAuto() {
    return new SequentialCommandGroup(
      new ConditionalCommand(
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180)))),
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
        this::isBlue
      )
    );
  }

  /**
   * 5 piece auto test PLACE HOLDER
   */
  private SequentialCommandGroup getTest5() {
    return new SequentialCommandGroup(
      // setup odometry
      new ConditionalCommand(
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180)))),
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
        this::isBlue
      ),
      // drive to reef
      new ParallelCommandGroup(
        AutoPlaceFactory.getAutoPlace(Constants.Elevator.Position.L3)
      ),
      // Drive to HP/Object
      new ParallelCommandGroup(
      )
    );
  }

}