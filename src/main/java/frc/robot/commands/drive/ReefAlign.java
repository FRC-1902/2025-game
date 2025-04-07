package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.WaypointType;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ReefAlign extends Command {
  private final SwerveSubsystem swerve;
  private int poleIndex = -1;
  private boolean poleChanged = false;
  
  private final PIDController pidX;
  private final PIDController pidY;
  private Pose2d targetPose;
  
  private final double distanceErrorTolerance = 0.06; // meters
  private final double rotationErrorTolerance = Math.toRadians(3); // degrees
  private final double maxVelocity = 4.0; // m/s
  private double currentDistance;
  private double currentRotError;

  /** Creates a new ReefAlign. */
  public ReefAlign(SwerveSubsystem swerve) {
    this.swerve = swerve;
    
    // Match PID values from SnapToWaypoint
    this.pidX = new PIDController(2.5, 0.02, 0.001);
    this.pidY = new PIDController(2.5, 0.02, 0.001);

    pidX.reset();
    pidY.reset();
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  /**
   * Navigate to a left (odd) or right (even) pole
   * @param goRight true for right/even poles, false for left/odd poles
   */
  public void navigateToDirectionalPole(boolean goRight) {
    // Get all poles
    Pose2d[] allPoles = swerve.allianceFlip(FieldConstants.WAYPOINTS.getReefPositions(0));
    
    // Find the nearest pole in the desired direction
    Translation2d robotPos = swerve.getPose().getTranslation();
    int targetIndex = -1;
    double minDistance = Double.MAX_VALUE;
    
    for (int i = 0; i < allPoles.length; i++) {
      boolean isRightPole = (i % 2 == 0);
      if (isRightPole == goRight) {
        // Calculate distance manually for reliability
        double dx = allPoles[i].getX() - robotPos.getX();
        double dy = allPoles[i].getY() - robotPos.getY();
        double dist = Math.sqrt(dx*dx + dy*dy);
        
        if (dist < minDistance) {
          minDistance = dist;
          targetIndex = i;
        }
      }
    }
    
    if (targetIndex >= 0) {
      poleIndex = targetIndex;
      poleChanged = true;
      SmartDashboard.putNumber("Reef/TargetPoleIndex", poleIndex);
      DataLogManager.log("Navigated to " + (goRight ? "right" : "left") + " pole index: " + poleIndex);
    }
  }
  
  /**
   * Find the closest pole regardless of left/right
   */
  private void findClosestPole() {
    Pose2d[] allPoles = swerve.allianceFlip(FieldConstants.WAYPOINTS.getReefPositions(0));
    Translation2d robotPos = swerve.getPose().getTranslation();
    
    double minDistance = Double.MAX_VALUE;
    for (int i = 0; i < allPoles.length; i++) {
      // Calculate distance manually for reliability
      double dx = allPoles[i].getX() - robotPos.getX();
      double dy = allPoles[i].getY() - robotPos.getY();
      double dist = Math.sqrt(dx*dx + dy*dy);
      
      if (dist < minDistance) {
        minDistance = dist;
        poleIndex = i;
      }
    }
    
    SmartDashboard.putNumber("Reef/ClosestPoleIndex", poleIndex);
    SmartDashboard.putNumber("Reef/ClosestPoleDistance", minDistance);
  }
  
  /**
   * Reset pole index to find the closest pole on next initialization
   */
  public void resetPoleIndex() {
    poleIndex = -1;
    SmartDashboard.putNumber("Reef/TargetPoleIndex", -1);
  }
  
  /**
   * Updates the target pose based on current pole index
   */
  private void updateTargetPose() {
    if (poleIndex >= 0) {
      // Get pole array with offset (final poses)
      Pose2d[] allOffsetPoles = swerve.allianceFlip(FieldConstants.WAYPOINTS.getReefPositions(FieldConstants.OFFSET));
      
      if (poleIndex < allOffsetPoles.length) {
        // Use the selected pole
        targetPose = allOffsetPoles[poleIndex];
      } else {
        // Fallback if pole index is invalid
        targetPose = swerve.getWaypoint(WaypointType.REEF, FieldConstants.OFFSET);
      }
    } else {
      // Fallback if no pole is selected
      targetPose = swerve.getWaypoint(WaypointType.REEF, FieldConstants.OFFSET);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Find the closest pole on initialization if no pole is selected
    if (poleIndex < 0) {
      findClosestPole();
    }
    
    // Reset PID controllers
    pidX.reset();
    pidY.reset();
    
    // Update target pose based on selected pole
    updateTargetPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (poleChanged) {
      updateTargetPose();
      poleChanged = false;
    }
    
    // Current robot pose
    Pose2d currentPose = swerve.getPose();
    
    // Calculate distance and rotation error (exactly like SnapToWaypoint)
    currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    currentRotError = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());
    
    SmartDashboard.putNumber("Auto/ReefDistance", currentDistance);
    SmartDashboard.putNumber("Auto/ReefRotError", Math.toDegrees(currentRotError));
    
    // Calculate velocity with PID (exactly like SnapToWaypoint)
    double xVelocity = pidX.calculate(currentPose.getX(), targetPose.getX());
    double yVelocity = pidY.calculate(currentPose.getY(), targetPose.getY());
    
    // Cap velocity (exactly like SnapToWaypoint)
    Translation2d velocity = new Translation2d(xVelocity, yVelocity);
    Translation2d cappedVelocity = velocity;
    
    double v = velocity.getDistance(Translation2d.kZero);
    if (v > maxVelocity) {
      cappedVelocity = cappedVelocity.times(1.0 / v).times(maxVelocity);
    }
    
    // Calculate rotation (exactly like SnapToWaypoint)
    double rotationkP = 4;
    Rotation2d rotation = targetPose.getRotation().minus(currentPose.getRotation()).times(rotationkP);
    double cappedRotation = Math.max(Math.min(rotation.getRadians(), 3), -3);
    
    // Drive to target
    swerve.drive(cappedVelocity, cappedRotation, true);
    
    // Vibrate controller when close to target
    if (currentDistance < distanceErrorTolerance && currentRotError < rotationErrorTolerance) {
      ControllerSubsystem.getInstance().vibrate(ControllerName.DRIVE, 100, 1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    swerve.drive(new Translation2d(0, 0), 0, true);
    DataLogManager.log("Reef alignment ended. Interrupted: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}