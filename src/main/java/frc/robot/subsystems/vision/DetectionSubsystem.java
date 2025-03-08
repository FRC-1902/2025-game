// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.OpenCVHelp;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
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
  private double xDistance;

  Matrix<N3,N3> cameraMatrix;
  Matrix<N8,N1> distCoeffs; 

  public void getUndistortedPoints(List<Point> corners) {
    Point[] cornerArray = corners.toArray(new Point[0]);

    Point[] undistortedArray = OpenCVHelp.undistortPoints(cameraMatrix, distCoeffs, cornerArray);

    for (int i = 0; i < corners.size(); i++) {
        corners.get(i).x = undistortedArray[i].x;
        corners.get(i).y = undistortedArray[i].y;
    }
  }

  public Point getTargetPoint(PhotonTrackedTarget currentObject) {
    if (currentObject == null) return null;

    var corners = currentObject.getMinAreaRectCorners();

    double y = Double.NEGATIVE_INFINITY;
    double sumX = 0.0;
    // double sumY = 0.0; // center point

    if (corners.size() == 0) return null;

    for (var corner : corners) {
      sumX += corner.x;
      // sumY += corner.y; // center point
      if (corner.y > y) {
          y = corner.y;
      }
    }

    double x = sumX / corners.size();
    // double y = sumY / corners.size(); // center point
    
    x = Math.max(0, Math.min(x, ObjectDetection.HORIZONTAL_RES));
    y = Math.max(0, Math.min(y, ObjectDetection.VERTICAL_RES));


    double normX = x/ObjectDetection.HORIZONTAL_RES;
    double normY = (ObjectDetection.VERTICAL_RES - y)/ObjectDetection.VERTICAL_RES;

    return new Point(normX, normY);
  }

  public double depth(Point point) {
    // Offset from center of the camera
    double angleOffset = point.y - .5;
    double pitchDeg =  angleOffset * (ObjectDetection.VERTICAL_FOV).getDegrees(); 

    double totalAngleDeg = (ObjectDetection.ANGLE).getDegrees() + pitchDeg;
    double totalAngleRad = Math.toRadians(totalAngleDeg);

    double distance = ObjectDetection.HEIGHT * Math.tan(totalAngleRad);  

    SmartDashboard.putNumber("VisionObjectDetection/angleOffset", angleOffset);
    SmartDashboard.putNumber("VisionObjectDetection/PitchDeg", pitchDeg);
    SmartDashboard.putNumber("VisionObjectDetection/TotalAngleDeg", totalAngleDeg);
    SmartDashboard.putNumber("VisionObjectDetection/Distance", distance);

    return distance;
  }

  public double width(Point point) {
    // Offset from center of the camera
    double angleOffset = point.x - .5;
    
    double yawDeg = angleOffset * (ObjectDetection.HORIZONTAL_FOV).getDegrees();
    double yawRad = Math.toRadians(yawDeg);
    
    double depth = depth(point);
    
    double xDistance = depth * Math.tan(yawRad)*2;
    
    SmartDashboard.putNumber("VisionObjectDetection/XangleOffset", angleOffset);
    SmartDashboard.putNumber("VisionObjectDetection/YawDeg", yawDeg);
    SmartDashboard.putNumber("VisionObjectDetection/xDistance", xDistance);
    
    return xDistance;
  }

  Transform2d cameraToObject = new Transform2d(
    new Translation2d(2.5, 0),
    new Rotation2d() 
  );


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
          distance = depth(targetPoint);
          xDistance = width(targetPoint);
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
