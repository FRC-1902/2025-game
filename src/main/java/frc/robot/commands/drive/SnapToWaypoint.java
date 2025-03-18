package frc.robot.commands.drive;


import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SnapToWaypoint extends Command {
  private final SwerveSubsystem swerve;
  private Supplier<Pose2d> targetPoseSupplier;
  private Pose2d targetPose;
  private PIDController pidX;
  private PIDController pidY;

  /**
   * snaps to specified targetPose based on currentPose
   * @param swerve
   * @param targetPoseSupplier
   */
  public SnapToWaypoint(SwerveSubsystem swerve, Supplier<Pose2d> targetPoseSupplier) {
    this.swerve = swerve;
    this.targetPoseSupplier = targetPoseSupplier;
    this.pidX = new PIDController(2.8, 0.002, 0.0);
    this.pidY = new PIDController(2.8, 0.002, 0.0);

    pidX.reset();
    pidY.reset();

    addRequirements(swerve);
  }


  @Override
  public void initialize() {
    targetPose = targetPoseSupplier.get();
  }

  @Override
  public void execute() {
     SmartDashboard.putData("PID/SnapToWaypointX", pidX);
     SmartDashboard.putData("PID/SnapToWaypointY", pidY);
    // Current robot pose
    Pose2d currentPose = swerve.getPose();

    // Calculate error
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();

    double xVelocity = -pidX.calculate(xError, 0);
    double yVelocity = -pidY.calculate(yError, 0);

    Translation2d cappedVelocity = new Translation2d(xVelocity, yVelocity);

    double rotationkP = 3; 
    Rotation2d rotation = targetPose.getRotation().minus(currentPose.getRotation()).times(rotationkP);
    double cappedRotation = Math.max(Math.min(rotation.getRadians(), 3), -3);
    
    swerve.drive(cappedVelocity, cappedRotation, true);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, true);
  }

  @Override
  public boolean isFinished() {
    // Finish when position and orientation are close enough
    double distanceThreshold = 0.05; // meters
    double rotationThreshold = Math.toRadians(1); // radians

    double currentDistance = swerve.getPose().getTranslation().getDistance(targetPose.getTranslation());
    double currentRotError = Math.abs(swerve.getPose().getRotation().getRadians() - targetPose.getRotation().getRadians());

    boolean positionReached = (currentDistance < distanceThreshold);
    boolean rotationReached = (currentRotError < rotationThreshold);

    return positionReached && rotationReached;
  }
}
