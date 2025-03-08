// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.OpenCVHelp;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision.ObjectDetection;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.estimation.OpenCVHelp;
import frc.robot.subsystems.swerve.SwerveSubsystem;



public class DetectionSubsystem extends SubsystemBase {
  /** Creates a new DetectionSubsystem. */
  private final PhotonCamera camera = new PhotonCamera(ObjectDetection.CAMERA_NAME);
  private final SwerveSubsystem swerve;

  // For debugging
  private boolean targetVisible = false;
  private double targetYaw;
  private double bottomDistance;
  private PhotonTrackedTarget currentObject;
  private Point targetPoint;
  private double distance;
  private double xDistance;

  public DetectionSubsystem(SwerveSubsystem swerve) {
    this.swerve = swerve;
  }


  public Point getTargetPoint(PhotonTrackedTarget currentObject) {
    if (currentObject == null) return null;
    
    Optional<Matrix<N3, N3>> cameraMatrixOpt = camera.getCameraMatrix();
    Optional<Matrix<N8, N1>> distCoeffsOpt = camera.getDistCoeffs();

        // Check if both camera matrix and distortion coefficients are available
        if (!cameraMatrixOpt.isPresent() || !distCoeffsOpt.isPresent()) {
          // Log error or handle the case when camera calibration data is not available
          System.err.println("Camera calibration data not available");
          return null;
      }
      
      Matrix<N3, N3> cameraMatrix = cameraMatrixOpt.get();
      Matrix<N8, N1> distCoeffs = distCoeffsOpt.get();
  

    Point[] cornersList = OpenCVHelp.cornersToPoints(currentObject.getMinAreaRectCorners());
    Point[] undistortedArray = OpenCVHelp.undistortPoints(cameraMatrix, distCoeffs, cornersList);

    List<Point> corners = Arrays.asList(cornersList);

    for (int i = 0; i < corners.size(); i++) {
        corners.get(i).x = undistortedArray[i].x;
        corners.get(i).y = undistortedArray[i].y;
    }
  
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
    
    double yawDeg = targetYaw; //angleOffset * (ObjectDetection.HORIZONTAL_FOV).getDegrees();
    double yawRad = Math.toRadians(yawDeg);
    
    double depth = distance;
    
    double xDistance = depth * Math.tan(yawRad) * 1.6;
    
    SmartDashboard.putNumber("VisionObjectDetection/XangleOffset", angleOffset);
    SmartDashboard.putNumber("VisionObjectDetection/YawDeg", yawDeg);
    SmartDashboard.putNumber("VisionObjectDetection/YawRad", yawRad);
    SmartDashboard.putNumber("VisionObjectDetection/xDistance", xDistance);
    
    return xDistance;
  }

  public Pose2d objectPose(double yawDegrees) {
    Pose2d robotPose = swerve.getPose();

    double yawRad = Math.toRadians(yawDegrees);

    double xCamera = distance * Math.cos(yawRad);  // forward
    double yCamera = distance * Math.sin(yawRad);  // left

    Transform2d cameraToObject = new Transform2d(
        new Translation2d(xCamera, yCamera),
        new Rotation2d()
    );

    Pose2d objectPose = robotPose.transformBy(cameraToObject);
    return objectPose;
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

  public Pose2d CartesianToPolar(Pose2d pose) {
    return new Pose2d(
      Math.sqrt(Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2)),
      Math.atan2(pose.getY(), pose.getX()),
      pose.getRotation()
    );
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
    SmartDashboard.putData("VisionObjectDetection/Object Pose", objectPose(targetYaw));
    
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
