package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DetectionSubsystem extends SubsystemBase {
  /** Creates a new DetectionSubsystem. */
  private final PhotonCamera camera = new PhotonCamera("colorCam");
  private boolean targetVisible;
  private boolean validTargetVisible;
  private double targetYaw; 
  private double targetPitch; 
  private double detectionTimeout = 0.5;
  private double lastValidDetectionTime;
  
  private int currentlyTrackedId = -1;
  private double lastIdSwitchTime = 0;
  private double idSwitchTimeout = 1.0;


  public DetectionSubsystem() {}

  /**
   * 
   * @returns if object is visible or not 
   */
  private boolean isTargetVisible(){
    return targetVisible;
  }

  /**
   * 
   * @returns if object is visible or not (post filtering)
   */
  public boolean isValidTargetVisible(){
    double currentTime = Timer.getFPGATimestamp();

    return validTargetVisible || (currentTime - lastValidDetectionTime < detectionTimeout);
  }

  /**
   * 
   * @returns the specific targetYaw of the object in Rotation2d's
   */
  public Rotation2d getTargetYaw(){
    return Rotation2d.fromDegrees(targetYaw);
  }

  /**
   * 
   * @returns the specific targetYaw of the object in Rotation2d's
   */
  public Rotation2d getTargetPitch(){
    return Rotation2d.fromDegrees(targetPitch);
  }

  // Gets the lowest ID
  @Override
  public void periodic() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    double currentTime = Timer.getFPGATimestamp();


    if (!results.isEmpty()) {
      // Get the oldest unread result
      PhotonPipelineResult result = results.get(results.size() - 1);
  
      if (result.hasTargets()) {
        boolean seenValidTarget = false;
        targetVisible = true;
        PhotonTrackedTarget bestTarget = null;

        // Checks if there was a previous target
        if (currentlyTrackedId >= 0) {
          for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == currentlyTrackedId && target.getPitch() >= -8) {
              bestTarget = target;
              seenValidTarget = true;
              break; // Found our previous target, stop searching
            }
          }
        }

        // If no previous target get lowest ID
        if (bestTarget == null) {
          // Only switch targets after timeout
          boolean allowNewTarget = 
            currentlyTrackedId < 0 || (currentTime - lastIdSwitchTime) > idSwitchTimeout;
          
          if (allowNewTarget) {
            int lowestId = Integer.MAX_VALUE;
            
            // Find the target with the lowest valid ID
            for (PhotonTrackedTarget target : result.getTargets()) {
              int targetId = target.getFiducialId();
              double pitch = target.getPitch();
              
              if (pitch >= -8 && targetId < lowestId) {
                lowestId = targetId;
                bestTarget = target;
                seenValidTarget = true;
              }
            }
            
            // Update tracked ID
            if (seenValidTarget && lowestId != currentlyTrackedId) {
              currentlyTrackedId = lowestId;
              lastIdSwitchTime = currentTime;
            }
          }
        }

        // Update valid target
        if (bestTarget != null) {
          targetYaw = bestTarget.getYaw() + 2;
          targetPitch = bestTarget.getPitch();
          lastValidDetectionTime = currentTime;
          validTargetVisible = true;
        }
      } else {
        targetVisible = false;
        validTargetVisible = false;
      }
    }

    // If tracking times out, reset ID
    if (currentTime - lastValidDetectionTime > 2.0) {
      currentlyTrackedId = -1;
    }
    
    // Dashboard telemetry
    boolean timeoutValid = (currentTime - lastValidDetectionTime < detectionTimeout);
    SmartDashboard.putBoolean("VisionObjectDetection/RawTargetVisible", targetVisible);
    SmartDashboard.putBoolean("VisionObjectDetection/ValidTargetVisible", validTargetVisible || timeoutValid);
    SmartDashboard.putNumber("VisionObjectDetection/TargetYaw", targetYaw);
    SmartDashboard.putNumber("VisionObjectDetection/TargetPitch", targetPitch);
    SmartDashboard.putNumber("VisionObjectDetection/CurrentlyTrackedId", currentlyTrackedId);
    SmartDashboard.putNumber("VisionObjectDetection/TimeSinceLastDetection", currentTime - lastValidDetectionTime);
  }
}
