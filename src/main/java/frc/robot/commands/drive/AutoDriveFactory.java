package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.WaypointType;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;

public class AutoDriveFactory {
  private SwerveSubsystem swerve;
  private ObjectDetectionSubsystem objectDetectionSubsystem;

  public AutoDriveFactory(SwerveSubsystem swerve, ObjectDetectionSubsystem objectDetectionSubsystem) {
    this.swerve = swerve;
    this.objectDetectionSubsystem = objectDetectionSubsystem;
  }

  /**
   * Drives to the waypoint and snaps to it.
   * @return the command
   */
  public Command pathAndSnapCommand(WaypointType waypoint) {
    DataLogManager.log("Auto Driving");
    return new SequentialCommandGroup(
      new PathToWaypoint(() -> swerve.getWaypoint(waypoint, FieldConstants.pathOffset), swerve),
      new ContinuallySnapToWaypoint(swerve, () -> swerve.getWaypoint(waypoint, FieldConstants.offset))
    );
  }

  public Command pathAndSnapToObjectCommand() {
    DataLogManager.log("Auto Driving to Object");
    return new SequentialCommandGroup(
      new PathToWaypoint(() -> objectDetectionSubsystem.getClosestObject(), swerve),
      new ContinuallySnapToWaypoint(swerve, () -> objectDetectionSubsystem.getClosestObject())
    );
  }
}
