// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class DetectionSubsystem extends SubsystemBase {
  /** Creates a new DetectionSubsystem. */
  private final PhotonCamera camera = new PhotonCamera("object cam"); // TODO: only use object detection cameras
  public boolean targetVisible;
  public double targetYaw; 

  private PhotonTrackedTarget currentObject;

  public DetectionSubsystem() {
  }

  @Override
  public void periodic() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    currentObject = null; // XXX: maybe race condition here
    targetVisible = false;

    if (!results.isEmpty()) {
      // Get the oldest unread result
      PhotonPipelineResult result = results.get(results.size() - 1);

      if (result.hasTargets()) {
          for (var target : result.getTargets()) {
            
            if (target.getDetectedObjectConfidence() < 0.5) // TODO: add a confidence threashold constant
              continue;
            
            if (currentObject == null || currentObject.getDetectedObjectConfidence() < target.getDetectedObjectConfidence())
              currentObject = target;
              targetYaw = currentObject.getYaw(); 
              targetVisible = true;
          }
      }
    }
  }
}
