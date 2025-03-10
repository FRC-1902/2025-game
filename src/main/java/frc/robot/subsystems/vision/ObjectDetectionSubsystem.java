// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.lang.reflect.Array;
import java.util.ArrayList;
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

  public Pose2d[] objects = new Pose2d[0];

  private Point targetPoint = null;
  private boolean calibrationInitialized = false;


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
      calibrationInitialized = true;
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

  public Pose2d[] objects() {
    return objects;
  }

  @Override
  public void periodic() {
    if (!calibrationInitialized) {
      getCameraCalibrationData();
    }

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      objects = new Pose2d[0];
      return;
    }

    // Get the most recent frame
    PhotonPipelineResult result = results.get(results.size() - 1);
    if (!result.hasTargets()) {
      // No targets in this frame
      objects = new Pose2d[0];
      return;
    }
    List<Pose2d> foundPoses = new ArrayList<>();

    for (PhotonTrackedTarget target : result.getTargets()) {
      // Confidence check
      if (target.getDetectedObjectConfidence() < 0.5) {
        continue;
      }

      // Convert target to a "bottom-center" point, then to a field Pose2d
      Point point = getTargetPoint(target);
      if (point == null) {
        continue;
      }
      Pose2d objectPose = getObjectPose(point);
      if (objectPose == null) {
        continue;
      }

      // Add it to our list
      foundPoses.add(objectPose);
    }

    // 4) Convert to array and store
    objects = foundPoses.toArray(new Pose2d[0]);
    SmartDashboard.putNumberArray("Vision/Object Poses", Arrays.stream(objects).mapToDouble(pose -> pose.getTranslation().getX()).toArray());
  }
}
