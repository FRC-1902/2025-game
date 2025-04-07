package frc.robot.commands.drive;


import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SnapToWaypoint extends Command {
  private final SwerveSubsystem swerve;
  private Supplier<Pose2d> targetPoseSupplier;
  private Pose2d targetPose;
  private final PIDController pidX;
  private final PIDController pidY;
  private final double distanceErrorTolerance = 0.06; // meters
  private final double rotationErrorTolerance = Math.toRadians(3); // degrees
  private final double maxVelocity;
  private double currentDistance;
  private double currentRotError;

  public SnapToWaypoint(SwerveSubsystem swerve, Supplier<Pose2d> targetPoseSupplier) {
    this(swerve, targetPoseSupplier, 4.0); // Default max velocity is 4.0 m/s
  }

  /**
   * snaps to specified targetPose based on currentPose
   * @param swerve
   * @param targetPoseSupplier
   */
  public SnapToWaypoint(SwerveSubsystem swerve, Supplier<Pose2d> targetPoseSupplier, double maxVelocity) {
    this.swerve = swerve;
    this.targetPoseSupplier = targetPoseSupplier;
    this.pidX = new PIDController(2.5, 0.02, 0.001);
    this.pidY = new PIDController(2.5, 0.02, 0.001);
    this.maxVelocity = maxVelocity;

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

    // Finish when position and orientation are close enough
    currentDistance = swerve.getPose().getTranslation().getDistance(targetPose.getTranslation());
    currentRotError = Math.abs(swerve.getPose().getRotation().getRadians() - targetPose.getRotation().getRadians());

    SmartDashboard.putNumber("Auto/Snap Distance", currentDistance);
    SmartDashboard.putNumber("Auto/Snap Rot", currentRotError);

    // Current robot pose
    Pose2d currentPose = swerve.getPose();

    double xVelocity = pidX.calculate(currentPose.getX(), targetPose.getX());
    double yVelocity = pidY.calculate(currentPose.getY(), targetPose.getY());

    Translation2d velocity = new Translation2d(xVelocity, yVelocity);
    Translation2d cappedVelocity = velocity;

    double v = velocity.getDistance(Translation2d.kZero);
    if (v > maxVelocity) {
      cappedVelocity = cappedVelocity.times(1.0 / v).times(maxVelocity);
    }

    double rotationkP = 4; 
    Rotation2d rotation = targetPose.getRotation().minus(currentPose.getRotation()).times(rotationkP);
    double cappedRotation = Math.max(Math.min(rotation.getRadians(), 3), -3);
    
    swerve.drive(cappedVelocity, cappedRotation, true);

    if ((currentDistance < distanceErrorTolerance) && (currentRotError < rotationErrorTolerance)) {
      ControllerSubsystem.getInstance().vibrate(ControllerName.DRIVE, 100, 1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, true);
  }

  @Override
  public boolean isFinished() {
    boolean positionReached = (currentDistance < distanceErrorTolerance);
    boolean rotationReached = (currentRotError < rotationErrorTolerance);
    return positionReached && rotationReached;
  }
}
