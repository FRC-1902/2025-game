package frc.robot.commands.drive;


import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SnapToWaypoint extends Command {
  private final SwerveSubsystem swerve;
  private Supplier<Pose2d> targetPoseSupplier;
  private Pose2d targetPose;
  private PIDController pidX;
  private PIDController pidY;
  private double distanceError = 0.08; // meters
  private double rotationError = Math.toRadians(2); // degrees
  private double currentDistance;
  private double currentRotError;

  /**
   * snaps to specified targetPose based on currentPose
   * @param swerve
   * @param targetPoseSupplier
   */
  public SnapToWaypoint(SwerveSubsystem swerve, Supplier<Pose2d> targetPoseSupplier) {
    this.swerve = swerve;
    this.targetPoseSupplier = targetPoseSupplier;
    this.pidX = new PIDController(2, 0.0, 0.0);
    this.pidY = new PIDController(2, 0.0, 0.0);

    pidX.reset();
    pidY.reset();

    addRequirements(swerve);
  }


  @Override
  public void initialize() {
    targetPose = targetPoseSupplier.get();
    pidX.reset();
    pidY.reset();
  }

  @Override
  public void execute() {
    SmartDashboard.putData("PID/SnapToWaypointX", pidX);
    SmartDashboard.putData("PID/SnapToWaypointY", pidY);
    double maxVelocity = 4.0;

    // Finish when position and orientation are close enough
    currentDistance = swerve.getPose().getTranslation().getDistance(targetPose.getTranslation());
    currentRotError = Math.abs(swerve.getPose().getRotation().getRadians() - targetPose.getRotation().getRadians());

    // Current robot pose
    Pose2d currentPose = swerve.getPose();

    double xVelocity = -pidX.calculate(targetPose.getX(), currentPose.getX());
    double yVelocity = -pidY.calculate(targetPose.getY(), currentPose.getY());

    Translation2d velocity = new Translation2d(xVelocity, yVelocity);
    Translation2d cappedVelocity = velocity;

    double v = velocity.getDistance(Translation2d.kZero);
    if (v > maxVelocity) {
      cappedVelocity = cappedVelocity.times(1.0 / v).times(maxVelocity);
    }

    double rotationkP = 3; 
    Rotation2d rotation = targetPose.getRotation().minus(currentPose.getRotation()).times(rotationkP);
    double cappedRotation = Math.max(Math.min(rotation.getRadians(), 3), -3);
    
    swerve.drive(cappedVelocity, cappedRotation, true);

    if ((currentDistance < distanceError) && (currentRotError < rotationError)) {
      ControllerSubsystem.getInstance().vibrate(ControllerName.DRIVE, 100, 1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, true);
  }

  @Override
  public boolean isFinished() {
    boolean positionReached = (currentDistance < distanceError);
    boolean rotationReached = (currentRotError < rotationError);

    return positionReached && rotationReached;
  }
}
