package frc.robot.subsystems.vision;

import java.util.ArrayList;
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
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ObjectDetectionSubsystem extends SubsystemBase {
  // Configuration for object tracking
  private static final double POSITION_MATCH_THRESHOLD = 0.2;  // meters
  private static final double CONFIDENCE_GAIN = 0.1;  // How quickly confidence increases
  private static final double CONFIDENCE_DECAY = 0.3;  // How quickly confidence decreases per second
  private static final double MIN_CONFIDENCE = 0.1;  // Minimum confidence to keep tracking
  private static final double DISPLAY_CONFIDENCE = 0.3;  // Minimum confidence to display object
  private static final double MAX_AGE = 2.0;  // Maximum time to track without seeing
  private static final double FILTER_WEIGHT = 0.3;  // Weight for new measurements (0-1)
  private static final double CONFIDENCE = 0.1;  // Minimum confidence to consider a target valid, independent of photons confidence, photon comes first, if it passes will basically just get filtered twice
  private static final double MAX_TRACKED_VELOCITY = 0.5; // m/s - max velocity to allow an object to be considered for closestObject to avoid chasing fast moving objects
  
  private Pose2d lastTrackedObjectPose = null;
  
  /** Creates a new DetectionSubsystem. */
  private final PhotonCamera camera = new PhotonCamera(Vision.CAMERA_OBJECT.CAMERA_NAME);
  private final SwerveSubsystem swerveSubsystem;
  private Optional<Matrix<N3, N3>> cameraMatrixOpt;
  private Optional<Matrix<N8, N1>> distCoeffsOpt;

  // Array to store detected objects
  public TrackedObject[] objects = new TrackedObject[0];
  
  // Camera calibration state
  private boolean calibrationInitialized = false;
  private double lastCalibrationAttempt = 0;
  private static final double CALIBRATION_RETRY_INTERVAL = 1.0; // seconds
  
  // Storage for tracked objects
  private Map<Integer, TrackedObject> trackedObjects = new HashMap<>();
  private int nextTrackId = 0;

  // Exclusion points configuration
  private static final boolean ENABLE_EXCLUSION_POINTS = true; 
  private static final double EXCLUSION_TOLERANCE = 0.5; // meters - how close is "too close" to excluded points

  // List of positions to exclude (in meters, field coordinates)
  private List<Translation2d> exclusionPoints = new ArrayList<>();

  // Alert for calibration errors
  private Alert calibrationErrorAlert;

  ////////////// SIM ////////////////////
  private boolean simulationMode = true;
  private Pose2d simulatedCoralPose = new Pose2d(5, 5, new Rotation2d());
  private List<Pose2d> simulatedDetections = new ArrayList<>();


  public ObjectDetectionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    edu.wpi.first.cscore.CvSink cvSink = new edu.wpi.first.cscore.CvSink("temp");
    cvSink.close(); // We don't actually need to use it

    calibrationErrorAlert = new Alert("Vision/Detection/Camera calibration data not available", AlertType.kError);

    getCameraCalibrationData();
    initializeExclusionPoints();
  }

    /**
   * Initialize exclusion points - call this in the constructor
   */
  private void initializeExclusionPoints() {
    exclusionPoints.add(new Translation2d(1.2, 2.2)); 
    exclusionPoints.add(new Translation2d(1.2, 4.02)); 
    exclusionPoints.add(new Translation2d(1.2, 5.85)); 
    exclusionPoints.add(new Translation2d(16.333, 2.2)); 
    exclusionPoints.add(new Translation2d(16.333, 4.02)); 
    exclusionPoints.add(new Translation2d(16.333, 5.85)); 
  }

  //////////////////////// SIMULATION ONLY ////////////////////////
  public void enableSimulation(Translation2d position) {
    simulationMode = true;
    simulatedCoralPose = new Pose2d(position, new Rotation2d());
    simulatedDetections.clear();
    simulatedDetections.add(simulatedCoralPose);
    System.out.println("Simulation enabled: Coral at " + position.getX() + ", " + position.getY());
  }

  /**
   * Disable simulation mode and return to using camera data
   */
  public void disableSimulation() {
      simulationMode = false;
      simulatedDetections.clear();
      System.out.println("Simulation disabled");
  }

  /**
   * Move the simulated coral to a new position
   * @param position New position for the simulated coral
   */
  public void moveSimulatedCoral(Translation2d position) {
      simulatedCoralPose = new Pose2d(position, new Rotation2d());
      if (simulationMode) {
          simulatedDetections.clear();
          simulatedDetections.add(simulatedCoralPose);
      }
  }
  ////////////////////////////////////////////////////////////

  /**
   * Checks if a position is too close to any exclusion point
   */
  private boolean isTooCloseToExcludedPoint(Translation2d position) {
    if (!ENABLE_EXCLUSION_POINTS) {
      return false;
    }
    
    for (Translation2d point : exclusionPoints) {
      double distance = point.getDistance(position);
      if (distance <= EXCLUSION_TOLERANCE) {
        SmartDashboard.putString("Vision/Detection/ExcludedNear", 
            String.format("Point near (%.2f, %.2f)", point.getX(), point.getY()));
        return true;
      }
    }
    return false;
  }

  /**
   * Fetches the photonvision calibration results
   * @return calibration results
   */
  private void getCameraCalibrationData() {
    // TODO: fix up these matricies
    double[] mat = {914.4473225619788, 0.0, 593.3724508723684, 0.0, 914.7543494185871, 473.17413201554217, 0.0, 0.0, 1.0};
    double[] mat2 = {0.03913534503908854, -0.05779809768824109, 0.00011209689892050617, -0.0009032158840746022, 0.0029564913451770175, -0.001342718063004936, 0.0006578999028743138, -0.0010550436326518305};
    // double[] mat = {0,0,0,0,0,0,0,0,0};
    // double[] mat2 = {0,0,0,0,0,0,0,0};
    cameraMatrixOpt = Optional.of(new Matrix<>(Nat.N3(), Nat.N3(), mat));
    distCoeffsOpt = Optional.of(new Matrix<>(Nat.N8(), Nat.N1(), mat2));
    // cameraMatrixOpt = camera.getCameraMatrix();
    // distCoeffsOpt = camera.getDistCoeffs();
    
    boolean hasCalibrationData = cameraMatrixOpt.isPresent() && distCoeffsOpt.isPresent();
    if (hasCalibrationData) {
      System.out.println("Camera calibration data successfully loaded");
      calibrationInitialized = true; // Only set to true when we have data
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
      calibrationErrorAlert.set(true);
      return null;
    } else {
      calibrationErrorAlert.set(false);
    }
      
    Matrix<N3, N3> cameraMatrix = cameraMatrixOpt.get();
    Matrix<N8, N1> distCoeffs = distCoeffsOpt.get();
  
    Point[] cornersList = OpenCVHelp.cornersToPoints(currentObject.getMinAreaRectCorners());
    cornersList = OpenCVHelp.undistortPoints(cameraMatrix, distCoeffs, cornersList);
  
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
    
    x = Math.max(0, Math.min(x, Vision.CAMERA_OBJECT.HORIZONTAL_RES));
    y = Math.max(0, Math.min(y, Vision.CAMERA_OBJECT.VERTICAL_RES));

    double normX = x/Vision.CAMERA_OBJECT.HORIZONTAL_RES;
    double normY = y/Vision.CAMERA_OBJECT.VERTICAL_RES;

    // flip coordinates
    normX = 1 - normX;
    normY = 1 - normY;

    // SmartDashboard.putNumber("Vision/Detection/normX", normX);
    // SmartDashboard.putNumber("Vision/Detection/normY", normY);

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
    double pitchRad = angleOffset * Vision.CAMERA_OBJECT.VERTICAL_FOV.getRadians();

    double totalAngleRad = Vision.CAMERA_OBJECT.CAMERA_OBJECT_POS.getRotation().getY() + pitchRad;

    // Horizontal distance to the object on the floor
    double distance = Vision.CAMERA_OBJECT.CAMERA_OBJECT_POS.getZ() * Math.tan(totalAngleRad);  

    SmartDashboard.putNumber("Vision/Detection/distance", distance);

    return distance;
  }

  /**
   * Calculates the yaw to a point relative to the camera
   * @param point
   * @return
   */
  private Rotation2d getYaw(Point point) {
    // Convert from normalized coordinates to tangent space
    double normalizedX = point.x - 0.5; // -0.5 to 0.5
    
    // Calculate the tangent based on the camera's focal properties
    // For a camera with horizontal FOV of θ, the focal length in normalized units is:
    // f = 0.5 / tan(θ/2)
    double horizontalFOVRad = Vision.CAMERA_OBJECT.HORIZONTAL_FOV.getRadians();
    double focalLength = 0.5 / Math.tan(horizontalFOVRad / 2.0);
    
    // Calculate the yaw using arctan
    double yawRad = Math.atan(normalizedX / focalLength);
    
    return new Rotation2d(yawRad);
  }

  /**
   * Calculates the object pose relative to the field
   * @param point Normalized detection point
   * @return Object pose in field coordinates
   */
  private Pose2d getObjectPose(Point point) {
    if (point == null) return null;
  
    // STEP 1: Get depth (straight-line distance to object in camera's forward direction)
    double depth = getDepth(point);
    
    // STEP 2: Get yaw (horizontal angle to object)
    double yawDegrees = getYaw(point).getDegrees();
    SmartDashboard.putNumber("Vision/Detection/yaw", yawDegrees);
  
    double yawRad = Math.toRadians(yawDegrees);
    
    // For camera, +X is forward, +Y is left
    double xCamera = depth * Math.cos(yawRad); // Forward distance 
    double yCamera = depth * Math.sin(yawRad) * 1.1; // Side distance
      
    // Get camera position on robot
    Transform3d cameraPose = Vision.CAMERA_OBJECT.CAMERA_OBJECT_POS;
    
    // Create a 3D transform from camera to object
    Transform3d cameraToObject = new Transform3d(
      new Translation3d(xCamera, yCamera, 0), // Object is on floor
      new Rotation3d(0, 0, 0)
    );
  
    Translation3d robotTrans = new Translation3d(swerveSubsystem.getPose().getTranslation());
    Rotation3d robotRot = new Rotation3d(swerveSubsystem.getPose().getRotation());
  
    // STEP 6: Combine all transforms to get field coordinates
    Pose3d robotPose3d = new Pose3d(
      new Translation3d(robotTrans.getX()-Units.inchesToMeters(4.879), robotTrans.getY(), 0), 
      new Rotation3d(robotRot.getX(), robotRot.getY(), robotRot.getZ()+Units.degreesToRadians(180)));
    Pose3d objectPose3d = robotPose3d
      .transformBy(cameraToObject);  // Go from camera to object
    
    // Apply camera transform to get camera position in field coordinates
    Pose3d cameraPose3d = robotPose3d.transformBy(cameraPose);
    
    // Log camera position and orientation
    Logger.recordOutput("Vision/Detection/CameraPose", cameraPose3d);
    
    // Log final object position in field coordinates
    SmartDashboard.putNumber("Vision/Detection/ObjectFieldX", objectPose3d.getX());
    SmartDashboard.putNumber("Vision/Detection/ObjectFieldY", objectPose3d.getY());
    
    // Project back to 2D for navigation
    return objectPose3d.toPose2d();
  }

  public boolean isTargetVisible() {
    return objects != null && objects.length > 0;
  }

  /**
   * Returns the closest object to the robot
   * @return The closest Pose2d object, or null if no objects detected
   */
  public Pose2d getClosestObject() {
    // Log entry to method for debugging
    boolean hasObjects = (objects != null && objects.length > 0);
    SmartDashboard.putBoolean("Vision/ClosestObject/HasObjects", hasObjects);
    SmartDashboard.putNumber("Vision/ClosestObject/ObjectCount", (objects != null) ? objects.length : 0);
    
    if (!hasObjects) return null;
  
    // Get robot position
    Pose2d robotPose = swerveSubsystem.getPose();
    Translation2d robotTranslation = robotPose.getTranslation();
    SmartDashboard.putNumber("Vision/ClosestObject/RobotX", robotTranslation.getX());
    SmartDashboard.putNumber("Vision/ClosestObject/RobotY", robotTranslation.getY());
  
    // Find the closest object that's NOT too close to an exclusion point
    TrackedObject closestTrackedObject = null;
    double closestDistance = Double.MAX_VALUE;
    int objectsFilteredByExclusion = 0;
  
    for (TrackedObject trackedObject : objects) {
      // Get the pose from the tracked object
      Pose2d objectPose = trackedObject.pose;
      
      // Log each object position
      Logger.recordOutput("Vision/ClosestObject/ObjectPositions", getObjectPoses3d());
      
      // Check exclusion points
      boolean tooClose = isTooCloseToExcludedPoint(objectPose.getTranslation());
      if (tooClose) {
        objectsFilteredByExclusion++;
        continue;
      }

      if (objectPose == null) continue;

      if (trackedObject.getSpeed() > MAX_TRACKED_VELOCITY) continue;
      
      double distance = robotTranslation.getDistance(objectPose.getTranslation());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestTrackedObject = trackedObject;
      }
    }
    
    // Log exclusion stats
    SmartDashboard.putNumber("Vision/ClosestObject/FilteredByExclusion", objectsFilteredByExclusion);
    
    // Check if we found any valid object after filtering
    SmartDashboard.putBoolean("Vision/ClosestObject/FoundValid", closestTrackedObject != null);
    
    if (closestTrackedObject != null) {
      Pose2d objectPose = closestTrackedObject.pose;
      // Log the distance to closest object
      SmartDashboard.putNumber("Vision/ClosestObject/Distance", closestDistance);
      
      // Calculate angle from robot to coral
      Translation2d coralTranslation = objectPose.getTranslation();
      double dx = coralTranslation.getX() - robotTranslation.getX();
      double dy = coralTranslation.getY() - robotTranslation.getY();
      
      // Calculate an angle that points TOWARD the coral (not +PI)
      double angleRad = Math.atan2(dy, dx);
      // SmartDashboard.putNumber("Vision/ClosestObject/AngleToObject", Math.toDegrees(angleRad));
      
      // Create new Pose2d with correct rotation
      Pose2d resultPose = new Pose2d(
        objectPose.getX(),
        objectPose.getY(),
        new Rotation2d(angleRad+Math.PI) // Flip to point toward object
      );
      
      // Log additional info from the tracked object
      // SmartDashboard.putNumber("Vision/ClosestObject/Confidence", closestTrackedObject.confidence);
      // SmartDashboard.putNumber("Vision/ClosestObject/TimeSinceLastSeen", 
          // Timer.getFPGATimestamp() - closestTrackedObject.lastSeenTimestamp);
      SmartDashboard.putBoolean("Vision/ClosestObject/IsMoving", closestTrackedObject.isMoving());
      
      // Log the final object position
      // SmartDashboard.putNumber("Vision/ClosestObject/FinalX", resultPose.getX());
      // SmartDashboard.putNumber("Vision/ClosestObject/FinalY", resultPose.getY());
      // SmartDashboard.putNumber("Vision/ClosestObject/FinalRotDeg", resultPose.getRotation().getDegrees());
      
      return resultPose;
    }
    return null;
  }
  
  /**
   * Gets a consistently tracked object - will be the closest object that's near the previously tracked object
   * @return The consistently tracked object's pose or null if no valid object found
   */
  public Pose2d getTrackedObject() {
    // First, get the current closest object
    Pose2d closestObject = getClosestObject();
    
    // If we don't have any objects at all, reset tracking
    if (closestObject == null) {
        lastTrackedObjectPose = null;
        SmartDashboard.putBoolean("Vision/TrackedObject/HasTarget", false);
        return null;
    }
    
    // If we haven't tracked an object yet, start with the closest one
    if (lastTrackedObjectPose == null) {
        lastTrackedObjectPose = closestObject;
        SmartDashboard.putBoolean("Vision/TrackedObject/HasTarget", true);
        return closestObject;
    }
    
    // Check if closest object is close to the last tracked object
    double distanceToLast = lastTrackedObjectPose.getTranslation()
        .getDistance(closestObject.getTranslation());
    
    if (distanceToLast <= POSITION_MATCH_THRESHOLD) {
        // Update to the new position if it's within tolerance
        lastTrackedObjectPose = closestObject;
        SmartDashboard.putBoolean("Vision/TrackedObject/HasTarget", true);
        SmartDashboard.putBoolean("Vision/TrackedObject/Updated", true);
    } else {
        // Keep the old position if the closest object has changed too much
        SmartDashboard.putBoolean("Vision/TrackedObject/Updated", false);
    }
    
    SmartDashboard.putNumber("Vision/TrackedObject/X", lastTrackedObjectPose.getX());
    SmartDashboard.putNumber("Vision/TrackedObject/Y", lastTrackedObjectPose.getY());
    
    return lastTrackedObjectPose;
  }

  /**
   * Resets object tracking to start fresh with the closest object next time
   */
  public void resetTracking() {
    lastTrackedObjectPose = null;
  }

  /**
   * Updates object tracking with new detections
   * @param newDetections List of newly detected object poses
   * @param currentTime Current timestamp
   */
  private void updateTracking(List<Pose2d> newDetections, double currentTime) {
    // Match detections to existing tracks
    Set<Integer> matchedTracks = new HashSet<>(trackedObjects.size());
    
    for (Pose2d detection : newDetections) {
      int bestMatchId = -1;
      double bestDistance = POSITION_MATCH_THRESHOLD;
      
      // Find closest existing track - simplified matching
      for (Map.Entry<Integer, TrackedObject> entry : trackedObjects.entrySet()) {
        int id = entry.getKey();
        TrackedObject obj = entry.getValue();
        
        double distance = obj.pose.getTranslation().getDistance(detection.getTranslation());
        if (distance < bestDistance) {
          bestDistance = distance;
          bestMatchId = id;
        }
      }
      
      if (bestMatchId != -1) {
        // Update existing track
        TrackedObject obj = trackedObjects.get(bestMatchId);
        obj.updatePose(detection, currentTime, FILTER_WEIGHT);
        obj.confidence = Math.min(1.0, obj.confidence + CONFIDENCE_GAIN);
        matchedTracks.add(bestMatchId);
      } else {
        // Create new track
        trackedObjects.put(nextTrackId, new TrackedObject(detection, currentTime));
        matchedTracks.add(nextTrackId);
        nextTrackId++;
      }
    }
    
    // Update unmatched tracks
    Iterator<Map.Entry<Integer, TrackedObject>> it = trackedObjects.entrySet().iterator();
    
    // Count objects by type (for debugging)
    int movingCount = 0;
    int stationaryCount = 0;
    
    while (it.hasNext()) {
      Map.Entry<Integer, TrackedObject> entry = it.next();
      TrackedObject obj = entry.getValue();
      
      if (!matchedTracks.contains(entry.getKey())) {
        double timeSinceLastSeen = currentTime - obj.lastSeenTimestamp;
        
        // Decrease confidence over time
        obj.confidence -= timeSinceLastSeen * CONFIDENCE_DECAY;
        
        // Remove if confidence too low or too old
        if (obj.confidence < MIN_CONFIDENCE || timeSinceLastSeen > MAX_AGE) {
          it.remove();
          continue;
        }
      }
      
      // Keep tracking motion stats for debugging
      if (obj.isMoving()) {
        movingCount++;
      } else {
        stationaryCount++;
      }
    }
    
    // Log counts
    SmartDashboard.putNumber("Vision/Detection/TrackedMovingObjects", movingCount);
    SmartDashboard.putNumber("Vision/Detection/TrackedStationaryObjects", stationaryCount);
  }

  /**
   * Converts Pose2d array to Pose3d array for logging
   * @param objectsToConvert Array of Pose2d objects
   * @return Array of Pose3d objects
   */
  private Pose3d[] getObjectPoses3d() {
    if (objects == null || objects.length == 0) {
      return new Pose3d[0];
    }
  
    Pose3d[] poses3d = new Pose3d[objects.length];
    for (int i = 0; i < objects.length; i++) {
      // Convert Pose2d to Pose3d
      Pose2d pose = objects[i].pose;
      poses3d[i] = new Pose3d(
        pose.getX(), 
        pose.getY(), 
        0.06,  // Z coordinate at ground level
        new Rotation3d(0,0, pose.getRotation().getRadians() + Math.PI/2)
      );
    }
    return poses3d;
  }
  
  /**
   * Log additional visualization for moving objects
   */
  private void logVelocityVectors() {
    List<Pose3d> velocityMarkers = new ArrayList<>();
    
    for (TrackedObject obj : trackedObjects.values()) {
      if (obj.isMoving() && obj.confidence >= DISPLAY_CONFIDENCE) {
        // Create a pose that points in the direction of movement
        Translation2d velocity = obj.getVelocity();
        double heading = Math.atan2(velocity.getY(), velocity.getX());
        
        velocityMarkers.add(new Pose3d(
          obj.pose.getX(),
          obj.pose.getY(),
          0.06, // Slightly above objects
          new Rotation3d(0, 0, heading)
        ));
      }
    }
    
    // Log velocity vectors if any exist
    if (!velocityMarkers.isEmpty()) {
      Logger.recordOutput("Vision/Detection/VelocityVectors", 
        velocityMarkers.toArray(new Pose3d[0]));
    }
  }

  /**
   * Periodic method to be called every loop
   * @return The tracked objects
   */
  @Override
  public void periodic() {
    // Basic debugging
    SmartDashboard.putBoolean("Vision/Detection/CameraStatus", camera.isConnected());
    
    // Try to get calibration data if needed
    if (!calibrationInitialized) {
      getCameraCalibrationData();
    }
    
    // Skip vision processing if we don't have calibration
    if (!calibrationInitialized) {
      return;
    }
      
    List<Pose2d> detectedObjects = new ArrayList<>();
      
    // Process new detections
    if (simulationMode) {
      // Use simulated detections in simulation mode
      detectedObjects.addAll(simulatedDetections);
      SmartDashboard.putBoolean("Vision/Detection/SimulationActive", true);
  } else {
      // Process new detections from camera
      SmartDashboard.putBoolean("Vision/Detection/SimulationActive", false);
      List<PhotonPipelineResult> results = camera.getAllUnreadResults();
      if (!results.isEmpty()) {
          // Get the most recent frame
          PhotonPipelineResult result = results.get(results.size() - 1);
          
          if (result.hasTargets()) {
              // Process each target
              for (PhotonTrackedTarget target : result.getTargets()) {
                  // Rest of your existing target processing code
                  // ...
              }
          }
      }
  }
    
    // Update tracking with new detections
    updateTracking(detectedObjects, Timer.getFPGATimestamp()); // TODO: change to result time
    
    // Generate output objects based on tracked objects with sufficient confidence
    objects = trackedObjects.values().stream()
      .filter(obj -> obj.confidence >= DISPLAY_CONFIDENCE)
      .toArray(TrackedObject[]::new); // Changed to store TrackedObject directly
  
    // Keep velocity vectors for debug visualization
    logVelocityVectors();
    
    // Debug output
    SmartDashboard.putNumber("Vision/Detection/DetectionCount", detectedObjects.size());
    SmartDashboard.putNumber("Vision/Detection/TrackingCount", trackedObjects.size());
    SmartDashboard.putNumber("Vision/Detection/OutputCount", objects.length);
    
    // Log to AdvantageScope
    Logger.recordOutput("Vision/Detection/DetectedObjects", getObjectPoses3d());
  }
}