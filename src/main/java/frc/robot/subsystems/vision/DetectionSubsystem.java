// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision.ObjectDetection;

public class DetectionSubsystem extends SubsystemBase {
  /** Creates a new DetectionSubsystem. */
  private final PhotonCamera camera = new PhotonCamera(ObjectDetection.CAMERA_NAME);

  // For debugging
  private boolean targetVisible = false;
  private double targetYaw;
  private double bottomDistance;
  private PhotonTrackedTarget currentObject;

  public Transform2d getObjectBottom() {
  }

  public boolean isTargetVisible() {
    return targetVisible;
  }

  public double getBottomDistanceMeters() {
    return bottomDistance;
  }

  public double getTargetYawDegrees() {
    return targetYaw;
  }

  public Rotation2d getTargetYaw() {
    return Rotation2d.fromDegrees(targetYaw);
  }

  public PhotonTrackedTarget getCurrentObject() {
    return currentObject;
  }

  @Override
  public void periodic() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    currentObject = null;
    bottomDistance = Double.MAX_VALUE;


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
