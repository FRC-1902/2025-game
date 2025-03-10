// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.OpenCVHelp;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision.ObjectDetection;
import frc.robot.subsystems.swerve.SwerveSubsystem;



public class ObjectDetectionSubsystem extends SubsystemBase {
  /** Creates a new DetectionSubsystem. */
  private final PhotonCamera camera = new PhotonCamera(ObjectDetection.CAMERA_NAME);
  private final SwerveSubsystem swerveSubsystem;

  private Optional<Matrix<N3, N3>> cameraMatrixOpt = Optional.empty();
  private Optional<Matrix<N8, N1>> distCoeffsOpt = Optional.empty();

  private boolean targetVisible = false;
  private PhotonTrackedTarget currentObject;

  private double lastSeenTime = 0.0;
  private static final double LOST_TARGET_TIMEOUT = 0.5;
  private Pose2d lastGoodPose = null;

  private Point targetPoint = null;

  public ObjectDetectionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    getCameraCalibrationData();
  }

  /**
   * Fetches the photonvision calibration results
   * @return calibration results
   */
  private boolean getCameraCalibrationData() {
    cameraMatrixOpt = camera.getCameraMatrix();
    distCoeffsOpt = camera.getDistCoeffs();
    
    boolean hasCalibrationData = cameraMatrixOpt.isPresent() && distCoeffsOpt.isPresent();
    if (!hasCalibrationData) {
      System.err.println("Camera calibration data not available yet");
    }
    
    return hasCalibrationData;
  }

  /**
   * Converts a PhotonTrackedTarget to a undistorted Point on the bottom center of an object
   * Can return null
   * @param currentObject
   * @return
   */
  private Point getTargetPoint(PhotonTrackedTarget currentObject) {
    if (currentObject == null) return null;

    if (!cameraMatrixOpt.isPresent() || !distCoeffsOpt.isPresent()) {
      System.err.println("Camera calibration data not available");
      return null;
    }
      
    Matrix<N3, N3> cameraMatrix = cameraMatrixOpt.get();
    Matrix<N8, N1> distCoeffs = distCoeffsOpt.get();
  
    Point[] cornersList = OpenCVHelp.cornersToPoints(currentObject.getMinAreaRectCorners());
    Point[] undistortedArray = OpenCVHelp.undistortPoints(cameraMatrix, distCoeffs, cornersList);

    for (int i = 0; i < cornersList.length; i++) {
      cornersList[i].x = undistortedArray[i].x;
      cornersList[i].y = undistortedArray[i].y;
    }
  
    double y = Double.NEGATIVE_INFINITY;
    double sumX = 0.0;

    if (cornersList.length == 0) return null;

    // Find the average x value and the max y value
    for (Point corner : cornersList) {
      sumX += corner.x;
      // Find the max y value
      if (corner.y > y) {
        y = corner.y;
      }
    }

    // Average the x values
    double x = sumX / cornersList.length;
    
    x = Math.max(0, Math.min(x, ObjectDetection.HORIZONTAL_RES));
    y = Math.max(0, Math.min(y, ObjectDetection.VERTICAL_RES));

    double normX = x/ObjectDetection.HORIZONTAL_RES;
    double normY = (ObjectDetection.VERTICAL_RES - y)/ObjectDetection.VERTICAL_RES;

    return new Point(normX, normY);
  }

  /**
   * Calculates the depth to a point relative to the camera
   * @param point
   * @return
   */
  private double getDepth(Point point) {
    // Calculate the angle offset from the center of the camera
    double angleOffset = point.y - .5;

    // Calculate the pitch in degrees
    double pitchDeg =  angleOffset * (ObjectDetection.VERTICAL_FOV).getDegrees(); 

    double totalAngleDeg = (ObjectDetection.ANGLE).getDegrees() + pitchDeg;
    double totalAngleRad = Math.toRadians(totalAngleDeg);

    // Horizontal distance to the object on the floor
    double distance = ObjectDetection.HEIGHT * Math.tan(totalAngleRad);  

    return distance;
  }

  /**
   * Calculates the yaw to a point relative to the camera
   * @param point
   * @return
   */
  private Rotation2d getYaw(Point point) {
    // Calculate the angle offset from the center of the camera
    double angleOffset = point.x - .5;
    
    Rotation2d yaw = Rotation2d.fromDegrees(-angleOffset * (ObjectDetection.HORIZONTAL_FOV).getDegrees());
    
    return yaw;
  }

  /**
   * Calculates the object pose relative to the field
   * Can return null
   * @param point
   * @return
   */
  public Pose2d getObjectPose(Point point) {
    if (point == null) return null;

    double depth = getDepth(point);
    double yawDegrees = getYaw(point).getDegrees();
    double yawRad = Math.toRadians(yawDegrees);

    double xCamera = depth * Math.cos(yawRad);
    double yCamera = depth * Math.sin(yawRad);

    Transform2d cameraToObject = new Transform2d(
      new Translation2d(xCamera, yCamera), // in meters
      new Rotation2d()
    );

    Pose2d robotPose = swerveSubsystem.getPose();
    Pose2d cameraPose = robotPose.transformBy(Constants.Vision.ObjectDetection.CAMERA_POSE);
    Pose2d objectPose = cameraPose.transformBy(cameraToObject);
    return objectPose;
  }

  public boolean isTargetVisible() {
    return targetVisible;
  }

  public PhotonTrackedTarget getCurrentObject() {
    return currentObject;
  }

  private void checkTargetTimeout() {
    double now = Timer.getFPGATimestamp();
    double timeSinceLastSeen = now - lastSeenTime;
    if (timeSinceLastSeen > LOST_TARGET_TIMEOUT) {
      // Timed out
      targetVisible = false;
      lastGoodPose = null;
      SmartDashboard.putBoolean("Vision/TargetVisible", false);
    } else {
      // Not visible *this frame*, but not timed out => keep lastGoodPose
      targetVisible = true;
      if (lastGoodPose != null) {
        // Show the old pose
        SmartDashboard.putBoolean("Vision/TargetVisible", true);
        SmartDashboard.putNumber("Vision/ObjectPoseX", lastGoodPose.getX());
        SmartDashboard.putNumber("Vision/ObjectPoseY", lastGoodPose.getY());
        SmartDashboard.putNumber("Vision/ObjectPoseHeading",lastGoodPose.getRotation().getDegrees());
      }
    }
  }

  /**
   * Returns the best-known object pose, or null if we have none.
   */
  private Pose2d getLastGoodPose() {
    return lastGoodPose;
  }

  @Override
  public void periodic() {
    // 1) Get latest results
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      // No new results => check if we timed out
      checkTargetTimeout();
      return;
    }
    
    // 2) Grab the most recent result
    PhotonPipelineResult result = results.get(results.size() - 1);
    if (!result.hasTargets()) {
      // No targets => check if we timed out
      checkTargetTimeout();
      return;
    }

    // 3) Find valid target(s)
    PhotonTrackedTarget bestTarget = null;
    double highestConfidence = 0.5; // example threshold
    for (PhotonTrackedTarget tgt : result.getTargets()) {
      if (tgt.getDetectedObjectConfidence() > highestConfidence) {
        bestTarget = tgt;
        highestConfidence = tgt.getDetectedObjectConfidence();
      }
    }
    if (bestTarget == null) {
      // No valid target => maybe timed out
      checkTargetTimeout();
      return;
    }

    // 4) Compute the pose of this best target
    Point targetPoint = getTargetPoint(bestTarget); // your lens-undistort + bottom-center code
    if (targetPoint == null) {
      checkTargetTimeout();
      return;
    }

    // We have a valid target with a valid point
    targetVisible = true;
    
    // Calculate measurements
    double depthMeters = getDepth(targetPoint);
    Rotation2d yaw = getYaw(targetPoint);
    Pose2d objectFieldPose = getObjectPose(targetPoint);
    
    
    // Only publish pose data if we have a valid pose
    if (objectFieldPose != null) {
      // Log to AdvantageKit
      Pose3d pose3d = new Pose3d(
        new Translation3d(objectFieldPose.getX(), objectFieldPose.getY()+Units.inchesToMeters(4.5/2), .15), 
        new Rotation3d(0, Math.toRadians(90), 0)
      );
      Logger.recordOutput("Vision/Object", pose3d);
    }
  }
}
