package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;

/**
 * Helper class that provides coral alignment assistance
 */
public class CoralAlignmentHelper {
  private final SwerveSubsystem swerve;
  private final ObjectDetectionSubsystem objectDetectionSubsystem;
  private final PIDController pid;
  
  public CoralAlignmentHelper(SwerveSubsystem swerve, ObjectDetectionSubsystem objectDetectionSubsystem) {
    this.swerve = swerve;
    this.objectDetectionSubsystem = objectDetectionSubsystem;
    
    // Simple PID for rotation
    this.pid = new PIDController(0.5, 0.0, 0.0);
    pid.enableContinuousInput(-Math.PI, Math.PI);
  }
  
  /**
   * Calculate rotation assistance value based on coral position
   * @return Rotation adjustment value, or 0 if no coral detected
   */
  public double calculateRotationAssistance() {
    // Get the closest coral
    Pose2d closestCoral = objectDetectionSubsystem.getClosestObject();
    
    if (closestCoral == null) {
      SmartDashboard.putBoolean("Vision/Detection/Active", false);
      return 0.0;
    }
    
    SmartDashboard.putBoolean("Vision/Detection/Active", true);
    
    // Get target and current angles
    double targetAngle = closestCoral.getRotation().getRadians();
    double currentAngle = swerve.getPose().getRotation().getRadians();
    
    // Calculate angle error
    double angleError = targetAngle - currentAngle;
    angleError = Math.atan2(Math.sin(angleError), Math.cos(angleError)); // Normalize
    
    // Calculate rotation speed
    double rotationSpeed = pid.calculate(currentAngle, targetAngle);
    
    rotationSpeed = Math.max(-0.8, Math.min(0.8, rotationSpeed));
    
    // Log debug info
    SmartDashboard.putNumber("Vision/Detection/TargetAngle", Math.toDegrees(targetAngle));
    SmartDashboard.putNumber("Vision/Detection/CurrentAngle", Math.toDegrees(currentAngle));
    SmartDashboard.putNumber("Vision/Detection/AngleError", Math.toDegrees(angleError));
    SmartDashboard.putNumber("Vision/Detection/AssistValue", rotationSpeed);
    
    return rotationSpeed;
  }
}