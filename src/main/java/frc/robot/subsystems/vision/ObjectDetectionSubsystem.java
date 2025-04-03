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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
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
import frc.robot.Constants.Vision;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ObjectDetectionSubsystem extends SubsystemBase {
  // Configuration for object tracking
  private static final double POSITION_MATCH_THRESHOLD = 0.2;  // meters
  private static final double CONFIDENCE_GAIN = 0.1;  // How quickly confidence increases
  private static final double CONFIDENCE_DECAY = 0.5;  // How quickly confidence decreases per second
  private static final double MIN_CONFIDENCE = 0.1;  // Minimum confidence to keep tracking
  private static final double DISPLAY_CONFIDENCE = 0.3;  // Minimum confidence to display object
  private static final double MAX_AGE = 1.0;  // Maximum time to track without seeing
  private static final double FILTER_WEIGHT = 0.3;  // Weight for new measurements (0-1)
  private static final double MIN_VELOCITY = 0.05;  // Minimum velocity to consider object moving (m/s)
  
  /** Creates a new DetectionSubsystem. */
  private final PhotonCamera camera = new PhotonCamera(Vision.CAMERA_OBJECT.CAMERA_NAME);
  private final SwerveSubsystem swerveSubsystem;

  private Optional<Matrix<N3, N3>> cameraMatrixOpt = Optional.empty();
  private Optional<Matrix<N8, N1>> distCoeffsOpt = Optional.empty();

  // Array to store detected objects
  public Pose2d[] objects = new Pose2d[0];
  
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

  // Class to represent a tracked object with filtering
  private static class TrackedObject {
    public Pose2d pose;
    private Translation2d filteredTranslation;
    private Translation2d velocity;  // Velocity vector
    public double lastSeenTimestamp;
    public double confidence;
    private boolean isMoving;
    
    public TrackedObject(Pose2d pose, double timestamp) {
      this.pose = pose;
      this.filteredTranslation = pose.getTranslation();
      this.velocity = new Translation2d();  // Zero velocity initially
      this.lastSeenTimestamp = timestamp;
      this.confidence = 0.5;  // Initial confidence
      this.isMoving = false;
    }
    
    public void updatePose(Pose2d newPose, double currentTime, double filterWeight) {
      // Calculate time delta
      double dt = currentTime - lastSeenTimestamp;
      if (dt <= 0) dt = 0.02;  // Avoid division by zero
      
      // Previous position
      Translation2d oldPosition = filteredTranslation;
      
      // Apply position filter
      double newX = filteredTranslation.getX() * (1-filterWeight) + newPose.getX() * filterWeight;
      double newY = filteredTranslation.getY() * (1-filterWeight) + newPose.getY() * filterWeight;
      filteredTranslation = new Translation2d(newX, newY);
      
      // Track velocity for debugging but simplified
      if (dt > 0) {
        double vx = (filteredTranslation.getX() - oldPosition.getX()) / dt;
        double vy = (filteredTranslation.getY() - oldPosition.getY()) / dt;
        velocity = new Translation2d(vx, vy);
        
        // Flag as moving if velocity exceeds threshold
        isMoving = velocity.getNorm() > MIN_VELOCITY;
      }
      
      // Create new pose with filtered position but use rotation from new pose
      pose = new Pose2d(filteredTranslation, newPose.getRotation());
      lastSeenTimestamp = currentTime;
    }
    
    public boolean isMoving() {
      return isMoving;
    }
    
    public double getSpeed() {
      return velocity.getNorm();
    }
  }

    /**
   * Initialize exclusion points - call this in the constructor
   */
  private void initializeExclusionPoints() {
    // Example points - replace with your actual positions to avoid
    exclusionPoints.add(new Translation2d(1.2, 2.2)); 
    exclusionPoints.add(new Translation2d(0.0, 5.54));
    // Add more points as needed
  }

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
        SmartDashboard.putString("Vision/ExcludedNear", 
            String.format("Point near (%.2f, %.2f)", point.getX(), point.getY()));
        return true;
      }
    }
    
    return false;
  }

  public ObjectDetectionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    edu.wpi.first.cscore.CvSink cvSink = new edu.wpi.first.cscore.CvSink("temp");
    cvSink.close(); // We don't actually need to use it

    getCameraCalibrationData();
    initializeExclusionPoints();
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
    
    x = Math.max(0, Math.min(x, Vision.CAMERA_OBJECT.HORIZONTAL_RES));
    y = Math.max(0, Math.min(y, Vision.CAMERA_OBJECT.VERTICAL_RES));

    double normX = x/Vision.CAMERA_OBJECT.HORIZONTAL_RES;
    double normY = (Vision.CAMERA_OBJECT.VERTICAL_RES - y)/Vision.CAMERA_OBJECT.VERTICAL_RES;

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
    double pitchDeg = angleOffset * (Vision.CAMERA_OBJECT.VERTICAL_FOV).getDegrees(); 

    double totalAngleDeg = Units.radiansToDegrees(Vision.CAMERA_OBJECT.CAMERA_OBJECT_POS.getRotation().getY()) + pitchDeg;
    double totalAngleRad = Math.toRadians(totalAngleDeg);

    // Horizontal distance to the object on the floor
    double distance = Vision.CAMERA_OBJECT.CAMERA_OBJECT_POS.getZ() * Math.tan(totalAngleRad);  

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
    
    Rotation2d yaw = Rotation2d.fromDegrees(-angleOffset * (Vision.CAMERA_OBJECT.HORIZONTAL_FOV).getDegrees());
    
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

    // Create a 3D transform from camera to object
    Transform3d cameraToObject = new Transform3d(
      new Translation3d(xCamera, yCamera, 0), // Object is on floor
      new Rotation3d(0, 0, 0)
    );

    // Get robot's position from odometry
    Pose2d robotPose = swerveSubsystem.getPose();
    Pose3d robotPose3d = new Pose3d(
      robotPose.getX(), 
      robotPose.getY(), 
      0.0, 
      new Rotation3d(0, 0, robotPose.getRotation().getRadians())
    );
    
    // Apply transforms to get object position
    Pose3d objectPose3d = robotPose3d
      .transformBy(Vision.CAMERA_OBJECT.CAMERA_OBJECT_POS)
      .transformBy(cameraToObject);
    
    // Project back to 2D for navigation
    return new Pose2d(
      objectPose3d.getX(),
      objectPose3d.getY(),
      new Rotation2d(objectPose3d.getRotation().getZ())
    );
  }

  public boolean isTargetVisible() {
    return objects != null && objects.length > 0;
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

    // Find the closest object that's NOT too close to an exclusion point
    Pose2d closestObject = null;
    double closestDistance = Double.MAX_VALUE;

    for (Pose2d objectPose : objects) {
      // Skip objects too close to exclusion points
      if (isTooCloseToExcludedPoint(objectPose.getTranslation())) {
        continue;
      }
      
      double distance = robotTranslation.getDistance(objectPose.getTranslation());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestObject = objectPose;
      }
    }

    if (closestObject != null) {
      // Calculate angle from robot to coral
      Translation2d coralTranslation = closestObject.getTranslation();
      double dx = coralTranslation.getX() - robotTranslation.getX();
      double dy = coralTranslation.getY() - robotTranslation.getY();
      double angleRad = Math.atan2(dy, dx) + Math.PI;
      
      // Create new Pose2d with correct rotation
      closestObject = new Pose2d(
        closestObject.getX(),
        closestObject.getY(),
        new Rotation2d(angleRad)
      );
    }

    return closestObject;
  }

  /**
   * Updates object tracking with new detections
   * @param newDetections List of newly detected object poses
   * @param currentTime Current timestamp
   */
  private void updateTracking(List<Pose2d> newDetections, double currentTime) {
    // Match detections to existing tracks
    Set<Integer> matchedTracks = new HashSet<>();
    
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
    SmartDashboard.putNumber("Vision/MovingObjects", movingCount);
    SmartDashboard.putNumber("Vision/StationaryObjects", stationaryCount);
  }

  @Override
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();

    // Basic debugging
    SmartDashboard.putBoolean("Vision/CalibrationInitialized", calibrationInitialized);
    
    // Try to get calibration data if needed
    if (!calibrationInitialized && 
        (currentTime - lastCalibrationAttempt) > CALIBRATION_RETRY_INTERVAL) {
      lastCalibrationAttempt = currentTime;
      getCameraCalibrationData();
    }
    
    // Skip vision processing if we don't have calibration
    if (!calibrationInitialized) {
      return;
    }
      
    List<Pose2d> detectedObjects = new ArrayList<>();
      
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
          
          detectedObjects.add(objectPose);
        }
      }
    }
    
    // Update tracking with new detections
    updateTracking(detectedObjects, currentTime);
    
    // Generate output objects based on tracked objects with sufficient confidence
    List<Pose2d> outputObjects = new ArrayList<>();
    for (TrackedObject obj : trackedObjects.values()) {
      if (obj.confidence >= DISPLAY_CONFIDENCE) {
        outputObjects.add(obj.pose);
      }
    }
    
    // Update the output array
    objects = outputObjects.toArray(new Pose2d[0]);
    
    // Keep velocity vectors for debug visualization
    logVelocityVectors();
    
    // Debug output
    SmartDashboard.putNumber("Vision/DetectionCount", detectedObjects.size());
    SmartDashboard.putNumber("Vision/TrackingCount", trackedObjects.size());
    SmartDashboard.putNumber("Vision/OutputCount", objects.length);
    
    // Log to AdvantageScope
    Logger.recordOutput("Vision/DetectedObjects", getObjectPoses3d());
    
    // If objects are detected, output the first object's position
    if (objects.length > 0) {
      SmartDashboard.putNumber("Vision/Object0_X", objects[0].getX());
      SmartDashboard.putNumber("Vision/Object0_Y", objects[0].getY());
      
      // Distance from robot to object
      Pose2d robotPose = swerveSubsystem.getPose();
      double distance = robotPose.getTranslation().getDistance(objects[0].getTranslation());
      SmartDashboard.putNumber("Vision/Object0_Distance", distance);
    }
  }

  private Pose3d[] getObjectPoses3d() {
    if (objects == null || objects.length == 0) {
      return new Pose3d[0];
    }

    Pose3d[] poses3d = new Pose3d[objects.length];
    for (int i = 0; i < objects.length; i++) {
      // Convert Pose2d to Pose3d
      poses3d[i] = new Pose3d(
        objects[i].getX(), 
        objects[i].getY(), 
        0.06,  // Z coordinate at ground level
        new Rotation3d(0, 0, objects[i].getRotation().getRadians())
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
        double heading = Math.atan2(obj.velocity.getY(), obj.velocity.getX());
        
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
      Logger.recordOutput("Vision/VelocityVectors", 
        velocityMarkers.toArray(new Pose3d[0]));
    }
  }
}