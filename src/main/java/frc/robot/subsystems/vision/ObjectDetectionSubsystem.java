// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

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

  private static final double MAX_TRACKING_AGE = 0.5; // Maximum time to track an object without seeing it (seconds)
  private static final double MIN_TRACKING_CONFIDENCE = 0.7; // Minimum confidence to report an object
  private static final double POSITION_MATCH_THRESHOLD = 0.5; // Maximum distance in meters to consider the same object

  private double lastCalibrationAttempt = 0;
private static final double CALIBRATION_RETRY_INTERVAL = 1.0; // seconds


  // Map to track objects over time - key is a unique ID, value is tracked object data
  private Map<Integer, TrackedObject> trackedObjects = new HashMap<>();
  private int nextObjectId = 0;


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
        // Don't set calibrationInitialized - we need to keep trying
        return false;
    } else {
        System.out.println("Camera calibration data successfully loaded");
        calibrationInitialized = true; // Only set to true when we have data
        return true;
    }
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

  // Class to track object data over time
  private static class TrackedObject {
    public Pose2d pose;
    public double lastSeenTimestamp;
    public double confidence;
    
    public TrackedObject(Pose2d pose, double timestamp, double initialConfidence) {
        this.pose = pose;
        this.lastSeenTimestamp = timestamp;
        this.confidence = initialConfidence;
    }
  }

  /**
   * Returns the closest object to the robot
   * @return The closest Pose2d object, or null if no objects detected
   */
  public Pose2d getClosestObject() {
    if (objects == null || objects.length == 0) {
      return null;
    }
    
    // Get robot position
    Pose2d robotPose = swerveSubsystem.getPose();
    Translation2d robotTranslation = robotPose.getTranslation();
    
    // Find the closest object
    Pose2d closestObject = null;
    double closestDistance = Double.MAX_VALUE;
    
    for (Pose2d objectPose : objects) {
      double distance = robotTranslation.getDistance(objectPose.getTranslation());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestObject = objectPose;
      }
    }
    
    return closestObject;
  }

  /**
 * Updates object tracking based on current detections
 */
private void updateTracking(List<Pose2d> currentObjects, double currentTime) {
    // First, match current detections to existing tracks
    Set<Integer> matchedTracks = new HashSet<>();
    Set<Pose2d> unmatchedDetections = new HashSet<>(currentObjects);
    
    // Try to match each detection to an existing track
    for (Pose2d detection : currentObjects) {
        int bestMatchId = -1;
        double bestDistance = POSITION_MATCH_THRESHOLD;
        
        // Find the closest tracked object
        for (Map.Entry<Integer, TrackedObject> entry : trackedObjects.entrySet()) {
            int id = entry.getKey();
            TrackedObject obj = entry.getValue();
            
            // Skip already matched tracks
            if (matchedTracks.contains(id)) continue;
            
            // Calculate distance between detection and tracked object
            double distance = detection.getTranslation().getDistance(obj.pose.getTranslation());
            if (distance < bestDistance) {
                bestDistance = distance;
                bestMatchId = id;
            }
        }
        
        // Update the matched track
        if (bestMatchId != -1) {
            TrackedObject obj = trackedObjects.get(bestMatchId);
            // Update position (could use a Kalman filter for better smoothing)
            obj.pose = detection;
            obj.lastSeenTimestamp = currentTime;
            obj.confidence = Math.min(1.0, obj.confidence + 0.2); // Increase confidence
            
            matchedTracks.add(bestMatchId);
            unmatchedDetections.remove(detection);
        }
    }
    
    // Create new tracks for unmatched detections
    for (Pose2d detection : unmatchedDetections) {
        TrackedObject newObj = new TrackedObject(detection, currentTime, 0.5);
        trackedObjects.put(nextObjectId++, newObj);
    }
    
    // Update confidence for unmatched tracks and remove old ones
    Iterator<Map.Entry<Integer, TrackedObject>> it = trackedObjects.entrySet().iterator();
    while (it.hasNext()) {
        Map.Entry<Integer, TrackedObject> entry = it.next();
        TrackedObject obj = entry.getValue();
        
        if (!matchedTracks.contains(entry.getKey())) {
            // Decrease confidence for objects not seen in this frame
            double timeSinceLastSeen = currentTime - obj.lastSeenTimestamp;
            obj.confidence -= timeSinceLastSeen * 2.0; // Decrease faster for longer unseen objects
            
            // Remove if too old or confidence too low
            if (timeSinceLastSeen > MAX_TRACKING_AGE || obj.confidence <= 0) {
                it.remove();
            }
        }
    }
  }

  @Override
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();
    
    // Try to get calibration data if needed, with rate limiting
    if (!calibrationInitialized && 
        (currentTime - lastCalibrationAttempt) > CALIBRATION_RETRY_INTERVAL) {
        lastCalibrationAttempt = currentTime;
        getCameraCalibrationData();
        
        // If we still don't have calibration, skip the rest of processing
        if (!calibrationInitialized) {
            return;
        }
    }
    
    // Skip vision processing if we don't have calibration
    if (!calibrationInitialized) {
        return;
    }
      
      List<Pose2d> currentFrameObjects = new ArrayList<>();
      
      // Process new detections
      List<PhotonPipelineResult> results = camera.getAllUnreadResults();
      if (!results.isEmpty()) {
          // Get the most recent frame
          PhotonPipelineResult result = results.get(results.size() - 1);
          if (result.hasTargets()) {
              // Process each target
              for (PhotonTrackedTarget target : result.getTargets()) {
                  // Skip low-confidence detections
                  if (target.getDetectedObjectConfidence() < 0.5) {
                      continue;
                  }
                  
                  // Convert target to a "bottom-center" point, then to a field Pose2d
                  Point point = getTargetPoint(target);
                  if (point == null) continue;
                  
                  Pose2d objectPose = getObjectPose(point);
                  if (objectPose == null) continue;
                  
                  currentFrameObjects.add(objectPose);
              }
          }
      }
      
      // Update tracking based on current frame detections
      updateTracking(currentFrameObjects, currentTime);
      
      // Generate filtered output array
      List<Pose2d> filteredPoses = new ArrayList<>();
      for (TrackedObject obj : trackedObjects.values()) {
          // Only include objects that have sufficient confidence
          if (obj.confidence >= MIN_TRACKING_CONFIDENCE) {
              filteredPoses.add(obj.pose);
          }
      }
      
      // Update the objects array
      objects = filteredPoses.toArray(new Pose2d[0]);
      
      // Update target visibility state
      targetVisible = objects.length > 0;
      if (targetVisible) {
          // Find the closest object for targeting
          Pose2d closest = getClosestObject();
          // Update current object (keeping it simple for now)
          // In a more sophisticated system, you'd track each object individually
      }
      
      // Log to dashboard
      SmartDashboard.putNumberArray("Vision/Object Poses", 
          Arrays.stream(objects).mapToDouble(pose -> pose.getTranslation().getX()).toArray());
      SmartDashboard.putNumber("Vision/Object Count", objects.length);
  }
}
