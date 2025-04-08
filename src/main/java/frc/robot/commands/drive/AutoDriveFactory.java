package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
  public Command snapCommand(WaypointType waypoint) {
    return new SequentialCommandGroup(
      // new PathToWaypoint(() -> swerve.getWaypoint(waypoint, FieldConstants.pathOffset), swerve),
      new ContinuallySnapToWaypoint(swerve, () -> swerve.getWaypoint(waypoint, FieldConstants.OFFSET))
    );
  }

  public Command snapOffsetCommand(WaypointType waypoint) {
    return new SequentialCommandGroup(
      new SnapToWaypoint(swerve, ()-> FieldConstants.WAYPOINTS.getOffsetPose(swerve.getWaypoint(waypoint, 0), FieldConstants.PATH_OFFSET)),
      new ContinuallySnapToWaypoint(swerve, () -> swerve.getWaypoint(waypoint, FieldConstants.OFFSET))
    );
  }

  public Command autoPathOffsetCommand(Pose2d waypoint) {
    Pose2d offsetWaypoint = FieldConstants.WAYPOINTS.getOffsetPose(waypoint, FieldConstants.PATH_OFFSET);

    return new SequentialCommandGroup(
      new SnapToWaypoint(swerve, () -> swerve.allianceFlip(offsetWaypoint))
    );
  }

  public Command autoSnapOffsetCommand(Pose2d waypoint) {
    Pose2d offsetWaypoint = FieldConstants.WAYPOINTS.getOffsetPose(waypoint, FieldConstants.OFFSET);

    return new SequentialCommandGroup(
      new SnapToWaypoint(swerve, () -> swerve.allianceFlip(offsetWaypoint))
    );
  }

  /**
   * Drives to the waypoint and snaps to it.
   * @return the command
   */
  public Command bargeAlignCommand(WaypointType waypoint) {
    return new SequentialCommandGroup(
      new PathToWaypoint( ()-> FieldConstants.WAYPOINTS.getOffsetPose(swerve.getWaypoint(waypoint, 0), -FieldConstants.BARGE_OFFSET), swerve),
      new SnapToWaypoint(swerve, () -> FieldConstants.WAYPOINTS.getOffsetPose(swerve.getWaypoint(waypoint, 0), Units.inchesToMeters(12)-FieldConstants.BARGE_OFFSET), .5)
    );
  }

  /**
   * Drives to specified waypoint and snaps to it.
   * @return the command
   */
  public Command pathAndSnapCommand(Pose2d waypoint) {
    Pose2d offsetWaypoint = FieldConstants.WAYPOINTS.getOffsetPose(waypoint, FieldConstants.PATH_OFFSET);
    Pose2d finalWaypoint = FieldConstants.WAYPOINTS.getOffsetPose(waypoint, FieldConstants.OFFSET);

    return new SequentialCommandGroup(
      new PathToWaypoint(() -> swerve.allianceFlip(offsetWaypoint), swerve),
      new SnapToWaypoint(swerve, () -> swerve.allianceFlip(finalWaypoint))
    );
  }

public Command pathAndSnapCoralCommand() {
  DataLogManager.log("Auto Driving to Coral");

  return new SequentialCommandGroup(
    new SnapToWaypoint(
      swerve, 
      () -> {
        Pose2d coral = objectDetectionSubsystem.getClosestObject();
        if (coral == null) {
          return swerve.getPose();
        }
        
        // We have a valid coral
        SmartDashboard.putBoolean("Vision/CoralTracking", true);
        
        // Apply the offset and return
        return FieldConstants.WAYPOINTS.getOffsetPose(coral, -FieldConstants.INTAKE_OFFSET);
      },
      2
    )
  );
}
}
