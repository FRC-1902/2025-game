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
  private final PhotonCamera camera = new PhotonCamera("HD_Pro_Webcam_C920"); // TODO: only use object detection cameras
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

  @Override
  public void periodic() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    currentObject = null;

    if (!results.isEmpty()) {
      // Get the oldest unread result
      PhotonPipelineResult result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        targetVisible = true;

        // loop over all targets, grabbing the highest confidence one
        for (var target : result.getTargets()) {
          // cull based on camera confidence
          if (target.getDetectedObjectConfidence() < 0.5) // TODO: add a confidence threashold constant
            continue;
          
          if (currentObject == null || currentObject.getDetectedObjectConfidence() < target.getDetectedObjectConfidence())
            currentObject = target;

          targetYaw = currentObject.getYaw();
        }
        
      } else {
        targetVisible = false;
      }
    }

    SmartDashboard.putBoolean("VisionObjectDetection/TargetVisible", targetVisible);
    SmartDashboard.putNumber("VisionObjectDetection/TargetYaw", targetYaw);
  }
}
