// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  private double targetYaw; 

  private PhotonTrackedTarget currentObject;

  public DetectionSubsystem() {}

  /**
   * 
   * @returns if object is visible or not 
   */
  public boolean isTargetVisible(){
    return targetVisible;
  }

  /**
   * 
   * @returns the specific targetYaw of the object in Rotation2d's
   */
  public Rotation2d getTargetYaw(){
    return Rotation2d.fromDegrees(targetYaw);
  }

  // Gets the lowest ID
  @Override
  public void periodic() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    currentObject = null;
    int lowestId = Integer.MAX_VALUE; // Track the lowest ID seen

    if (!results.isEmpty()) {
      // Get the oldest unread result
      PhotonPipelineResult result = results.get(results.size() - 1);
  
      if (result.hasTargets()) {
        targetVisible = true;
  
        // Loop over all targets, finding the one with lowest ID that meets confidence threshold
        for (var target : result.getTargets()) {

          int targetId = target.getFiducialId();
          
          // Update if this is the lowest ID we've seen
          if (targetId < lowestId) {
            currentObject = target;
            lowestId = targetId;
            targetYaw = currentObject.getYaw();
          }
        }
      } else {
        targetVisible = false;
      }
    }

    SmartDashboard.putBoolean("VisionObjectDetection/TargetVisible", targetVisible);
    SmartDashboard.putNumber("VisionObjectDetection/TargetYaw", targetYaw);
  }
}
