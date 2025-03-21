package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class DetectionSubsystem extends SubsystemBase {
  /** Creates a new DetectionSubsystem. */
  private final PhotonCamera camera = new PhotonCamera("colorCam");
  private boolean targetVisible;
  private boolean validTargetVisible;
  private double targetYaw; 
  private double targetPitch; 


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
    return validTargetVisible;
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


    if (!results.isEmpty()) {
      // Get the oldest unread result
      PhotonPipelineResult result = results.get(results.size() - 1);
  
      if (result.hasTargets()) {
        boolean seenValidTarget = false;
        targetVisible = true;

        int lowestId = Integer.MAX_VALUE; // Track the lowest ID seen
  
        // Loop over all targets, finding the one with lowest ID that meets confidence threshold
        for (PhotonTrackedTarget target : result.getTargets()) {

          int targetId = target.getFiducialId();
          double pitch = target.getPitch(); 
          
          // Only consider targets above the minimum pitch threshold
          if (pitch >= -10 && targetId < lowestId) {
            lowestId = targetId;

            targetYaw = target.getYaw() + 2;
            targetPitch = pitch;
            seenValidTarget = true;
          }
        }
        validTargetVisible = seenValidTarget;
      } else {
        targetVisible = false;
        validTargetVisible = false;
      }
    }

    SmartDashboard.putBoolean("VisionObjectDetection/TargetVisible", targetVisible);
    SmartDashboard.putBoolean("VisionObjectDetection/ValidTargetVisible", validTargetVisible);
    SmartDashboard.putNumber("VisionObjectDetection/TargetYaw", targetYaw);
  }
}
