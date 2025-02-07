package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants.WaypointType;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoDriveFactory {
  private SwerveSubsystem swerve;

  public AutoDriveFactory(SwerveSubsystem swerve) {
    this.swerve = swerve;
  }

  /**
   * Drives to the waypoint and snaps to it.
   * 
   * @return the command
   */
  public Command pathAndSnapCommand(WaypointType waypoint) {
    return new SequentialCommandGroup(
      new PathToWaypoint(swerve, () -> swerve.getWaypoint(waypoint)),
      new SnapToWaypoint(swerve, () -> swerve.getWaypoint(waypoint))
    );
  }
}
