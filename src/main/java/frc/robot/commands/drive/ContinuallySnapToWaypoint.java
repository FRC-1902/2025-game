package frc.robot.commands.drive;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ContinuallySnapToWaypoint extends SnapToWaypoint {

  /**
   * Inherits SnapToWaypoint and runs continually
   * @param swerve subsystem
   * @param targetPoseSupplier as a Pose2d 
   */
  public ContinuallySnapToWaypoint(SwerveSubsystem swerve, Supplier<Pose2d> targetPoseSupplier) {
    super(swerve, targetPoseSupplier);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
