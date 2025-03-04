package frc.robot.commands.drive;


import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SnapToWaypoint extends Command {
  private final SwerveSubsystem swerve;
  private Supplier<Pose2d> targetPoseSupplier;
  private Pose2d targetPose;

  /** Creates a new SnapToWaypoint. */
  public SnapToWaypoint(SwerveSubsystem swerve, Supplier<Pose2d> targetPoseSupplier) {
    this.swerve = swerve;
    this.targetPoseSupplier = targetPoseSupplier;

    addRequirements(swerve);
  }


  @Override
  public void initialize() {
    targetPose = targetPoseSupplier.get();
  }

  @Override
  public void execute() {
    // Current robot pose
    Pose2d currentPose = swerve.getPose();

    // Simple P-controllers for translation and rotation
    double velocitykP = 0.001; // TODOL Tune these values
    double rotationkP = 0.001; // TODO: Tune these values

    Translation2d velocity = targetPose.getTranslation().minus(currentPose.getTranslation()).times(velocitykP);
    Rotation2d rotation = targetPose.getRotation().minus(currentPose.getRotation()).times(rotationkP);

    double cappedXVelocity = Math.max(Math.min(velocity.getX(), 1), -1); // TODO: Change cap
    double cappedYVelocity = Math.max(Math.min(velocity.getY(), 1), -1); // TODO: Change cap

    double cappedRotation = Math.max(Math.min(rotation.getRadians(), 1), -1); // TODO: Change cap
    
    Translation2d cappedVelocty = new Translation2d(cappedXVelocity, cappedYVelocity);

    swerve.drive(cappedVelocty, cappedRotation, true);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, true);
  }

  @Override
  public boolean isFinished() {
    // Finish when position and orientation are close enough
    double distanceThreshold = 0.03;        // meters
    double rotationThreshold = Math.toRadians(1); // radians

    double currentDistance = swerve.getPose().getTranslation().getDistance(targetPose.getTranslation());
    double currentRotError = Math.abs(swerve.getPose().getRotation().getRadians() - targetPose.getRotation().getRadians());

    boolean positionReached = (currentDistance < distanceThreshold);
    boolean rotationReached = (currentRotError < rotationThreshold);

    return positionReached && rotationReached;
  }
}
