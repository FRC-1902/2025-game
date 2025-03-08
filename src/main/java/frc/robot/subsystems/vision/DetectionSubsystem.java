// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.opencv.core.Point;
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
  private Point targetPoint;
  private double distance;

  public Point getTargetPoint(PhotonTrackedTarget currentObject) {
    if (currentObject == null) return null;

    var corners = currentObject.getMinAreaRectCorners();

    double y = Double.NEGATIVE_INFINITY;
    double sumX = 0.0;

    if (corners.size() == 0) return null;

    for (var corner : corners) {
      sumX += corner.x;
      if (corner.y > y) {
          y = corner.y;
      }
    }

    double x = sumX / corners.size();
    
    x = Math.max(0, Math.min(x, ObjectDetection.HORIZONTAL_RES));
    y = Math.max(0, Math.min(y, ObjectDetection.VERTICAL_RES));


    double normY = (ObjectDetection.VERTICAL_RES - y)/ObjectDetection.VERTICAL_RES;
    double normX = x/ObjectDetection.HORIZONTAL_RES;

    return new Point(normX, normY);
  }

  public double distance(Point point) {
    double angleOffset = point.y + .5;
    double pitchDeg =  angleOffset * (ObjectDetection.VERTICAL_FOV).getDegrees(); 

    double totalAngleDeg = (ObjectDetection.ANGLE).getDegrees() + pitchDeg;
    double totalAngleRad = Math.toRadians(totalAngleDeg);

    double distance = ObjectDetection.HEIGHT / Math.tan(totalAngleRad);  

    SmartDashboard.putNumber("Vision/pointY", point.y);
    SmartDashboard.putNumber("Vision/PitchDeg", pitchDeg);
    SmartDashboard.putNumber("Vision/TotalAngleDeg", totalAngleDeg);
    SmartDashboard.putNumber("Vision/Distance", distance);


    return distance;
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
          targetPoint = getTargetPoint(currentObject);
          distance = distance(targetPoint);
        }
        
      } else {
        targetVisible = false;
      }
    }

    SmartDashboard.putBoolean("VisionObjectDetection/TargetVisible", targetVisible);
    SmartDashboard.putNumber("VisionObjectDetection/TargetYaw", targetYaw);
    
    if (targetPoint != null) {
        SmartDashboard.putNumber("VisionObjectDetection/TargetX", targetPoint.x);
        SmartDashboard.putNumber("VisionObjectDetection/TargetY", targetPoint.y);
        SmartDashboard.putNumber("VisionObjectDetection/Distance", distance);
    } else {
        SmartDashboard.putNumber("VisionObjectDetection/TargetX", Double.NaN);
        SmartDashboard.putNumber("VisionObjectDetection/TargetY", Double.NaN);
        SmartDashboard.putNumber("VisionObjectDetection/Distance", Double.NaN);

    }
  }
}
