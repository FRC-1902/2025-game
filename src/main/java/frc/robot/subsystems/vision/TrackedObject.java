package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Class to represent a tracked object with filtering and motion detection
 */
public class TrackedObject {
  public Pose2d pose;
  private Translation2d filteredTranslation;
  private Translation2d velocity;  // Velocity vector
  public double lastSeenTimestamp;
  public double confidence;
  private boolean isMoving;
  private double distanceToRobot; 
  
  /**
   * Static configuration constants
   */
  private static final double MIN_VELOCITY = 0.1;  // Minimum velocity to consider object moving (m/s)
  
  /**
   * Creates a new tracked object
   * @param pose Initial pose of the object
   * @param timestamp Current timestamp when object was detected
   */
  public TrackedObject(Pose2d pose, double timestamp) {
    this.pose = pose;
    this.filteredTranslation = pose.getTranslation();
    this.velocity = new Translation2d();  // Zero velocity initially
    this.lastSeenTimestamp = timestamp;
    this.confidence = 0.2;  // Initial confidence
    this.isMoving = false;
    this.distanceToRobot = 0.0; 
  }
  
  /**
   * Updates the object's position with new measurement
   * @param newPose New detected position
   * @param currentTime Current timestamp
   * @param filterWeight Weight of new measurement (0-1)
   */
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

  /**
   * Sets the distance from robot to this object
   * @param distance Distance in meters
   */
  public void setDistanceToRobot(double distance) {
    this.distanceToRobot = distance;
  }

  /**
   * @return Distance from robot to this object in meters
   */
  public double getDistanceToRobot() {
    return distanceToRobot;
  }
  
  /**
   * @return Whether the object is considered to be in motion
   */
  public boolean isMoving() {
    return isMoving;
  }
  
  /**
   * @return Current speed in m/s
   */
  public double getSpeed() {
    return velocity.getNorm();
  }
  
  /**
   * @return The current velocity vector
   */
  public Translation2d getVelocity() {
    return velocity;
  }
}