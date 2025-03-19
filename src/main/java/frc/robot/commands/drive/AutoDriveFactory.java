package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.WaypointType;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoDriveFactory {
  private SwerveSubsystem swerve;

  public AutoDriveFactory(SwerveSubsystem swerve) {
    this.swerve = swerve;
  }

  /**
   * Drives to the waypoint and snaps to it.
   * @return the command
   */
  public Command snapCommand(WaypointType waypoint) {
    DataLogManager.log("Auto Driving");
    return new SequentialCommandGroup(
      // new PathToWaypoint(() -> swerve.getWaypoint(waypoint, FieldConstants.pathOffset), swerve),
      new ContinuallySnapToWaypoint(swerve, () -> swerve.getWaypoint(waypoint, FieldConstants.offset))
    );
  }

  /**
   * Drives to specified waypoint and snaps to it.
   * @return the command
   */
  public Command pathAndSnapCommand(Pose2d waypoint) {
    DataLogManager.log("Auto Driving");
    return new SequentialCommandGroup(
      new PathToWaypoint(() -> FieldConstants.WAYPOINTS.getOffsetPose(waypoint, FieldConstants.pathOffset), swerve),
      new SnapToWaypoint(swerve, () -> waypoint)
    );
  }

}
