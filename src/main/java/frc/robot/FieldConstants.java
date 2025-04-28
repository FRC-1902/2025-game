package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Constants for points on the field
 * Relative to the blue side origin
 */
public final class FieldConstants {

  public enum WaypointType {
    REEF,
    PROCESSOR,
    TROUGH,
    BARGE,
    HP
  }

  public static final double WIDTH = Units.inchesToMeters(317);
  public static final double LENGTH = Units.inchesToMeters(690.875);

  public static final double ROBOT_OFFSET_FRONT = Units.inchesToMeters(17.5); // Different distances to account for non-symmetrical robot
  public static final double ROBOT_OFFSET_BACK = Units.inchesToMeters(17);
  public static final double OFFSET = ROBOT_OFFSET_FRONT + Units.inchesToMeters(4.5);
  public static final double TELLY_OFFSET = ROBOT_OFFSET_FRONT + Units.inchesToMeters(2);
  public static final double PATH_OFFSET = ROBOT_OFFSET_FRONT + Units.inchesToMeters(2);
  public static final double BARGE_OFFSET = Units.inchesToMeters(50); 
  public static final double INTAKE_OFFSET = ROBOT_OFFSET_BACK + Units.inchesToMeters(-2); 

  public static final double TROUGH_OFFSET = Units.inchesToMeters(8);
  public static final double TROUGH_OUTWARD_DISTANCE = Units.inchesToMeters(19); // Distance from pole to trough
  public static final Rotation2d TROUGH_ANGLE_OFFSET = Rotation2d.fromDegrees(-15);

  static {
    SmartDashboard.putNumber("Field/Trough Offset", TROUGH_OFFSET);
    SmartDashboard.putNumber("Field/Trough Angle Offset", TROUGH_ANGLE_OFFSET.getDegrees()); 
    SmartDashboard.putNumber("Field/Trough Inward Distance", TROUGH_OUTWARD_DISTANCE);
  }

  public static double getTroughOffset() {
    return SmartDashboard.getNumber("Field/Trough Offset", TROUGH_OFFSET);
  }
  public static double getTroughOutwardDistance() {
    return SmartDashboard.getNumber("Field/Trough Outtake Distance", TROUGH_OUTWARD_DISTANCE);
  }
  
  public static Rotation2d getTroughAngleOffset() {
    // Convert from degrees in dashboard to radians for calculations
    return Rotation2d.fromDegrees(SmartDashboard.getNumber("Field/Trough Angle Offset", TROUGH_ANGLE_OFFSET.getDegrees()));
  }
  
  public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  
  public static class Reef {

    // Distance from center of face to each pole (left/right)
    public static final double POLE_SIDE_OFFSET = Units.inchesToMeters(6.469);
    
    // The faces of the hexagon, indexed counter-clockwise starting from driver station
    public static final Pose2d[] centerFaces = new Pose2d[6];

    static {
      // Initialize faces counter-clockwise starting from the driver station
      centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d(); // Face closest to driver station
      centerFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
      centerFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
      centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
      centerFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
      centerFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();
      
      // Create outward-facing versions (rotated 180 degrees)
      for (int i = 0; i < 6; i++) {
        Pose2d face = centerFaces[i];
        centerFaces[i] = new Pose2d(
          face.getX(), 
          face.getY(), 
          face.getRotation().plus(new Rotation2d(Math.PI)) // Add 180 degrees
        );
      }
    }
    
    /**
     * Calculates pole positions by offsetting left/right from AprilTag face centers
     * Requires centerFaces to be initialized
     */
    public static Pose2d[] calculatePoles() {
      Pose2d[] poles = new Pose2d[centerFaces.length * 2];
      
      for (int face = 0; face < centerFaces.length; face++) {
        Pose2d facePose = centerFaces[face]; // Use outward-facing poses
        
        // From the face center, offset to the right for the right pole (L3)
        poles[face * 2] = WAYPOINTS.getPerpendicularOffsetPose(facePose, POLE_SIDE_OFFSET, true);
        
        // From the face center, offset to the left for the left pole (L2)
        poles[face * 2 + 1] = WAYPOINTS.getPerpendicularOffsetPose(facePose, POLE_SIDE_OFFSET, false);
      }
      
      return poles;
    }
  }

  /* Algae Placements, Counter Clockwise, Start from DS closest face
   * 1: L3
   * 2: L2
   * 3: L3
   * 4: L2
   * 5: L3
   * 6: L2
   */

  public static final class WAYPOINTS {
    private WAYPOINTS() {}; 
    // Reef Waypoints, Lettering goes counterclockwise blue, clockwise red
    // TODO: Set real values
    public static final Pose2d PROCESSOR = new Pose2d(6.100, 0.780, Rotation2d.fromDegrees(270));

    public static final Pose2d[] POLES = Reef.calculatePoles();

    public static final Pose2d[] BARGE = {
      new Pose2d(8.775, 7.260, Rotation2d.fromDegrees(180)), // Left
      new Pose2d(8.775, 6.196, Rotation2d.fromDegrees(180)), // Mid
      new Pose2d(8.775, 5.078, Rotation2d.fromDegrees(180)) // Right
    };

    public static final Pose2d[] HP = {
      new Pose2d(3.000, 6.000, Rotation2d.fromDegrees(140-180)), // Left
      new Pose2d(3.000, 2.000, Rotation2d.fromDegrees(210-180)), // Right
    };

    public static void updatePolesFromAprilTags() {
      Pose2d[] updatedPoles = Reef.calculatePoles();
      for (int i = 0; i < POLES.length; i++) {
        POLES[i] = updatedPoles[i];
      }
    }

    public static Pose2d getOffsetPose(Pose2d pose, double offsetDistance) {
      double offsetX = pose.getX() - offsetDistance * Math.cos(pose.getRotation().getRadians());
      double offsetY = pose.getY() - offsetDistance * Math.sin(pose.getRotation().getRadians());
      return new Pose2d(offsetX, offsetY, pose.getRotation());
    }

    // Add a method that generates reef positions with a custom offset
    public static Pose2d[] getReefPositions(double offset) {
      Pose2d[] reefPositions = new Pose2d[POLES.length];
      for (int i = 0; i < POLES.length; i++) {
          reefPositions[i] = getOffsetPose(POLES[i], offset);
      }
      return reefPositions;
    }

    /**
     * Gets a pose that's offset perpendicular to the facing direction
     * @param pose The original pose
     * @param offsetDistance Distance to offset
     * @param toRight If true, offset to the right of facing direction; if false, offset to the left
     * @return The offset pose
     */
    public static Pose2d getPerpendicularOffsetPose(Pose2d pose, double offsetDistance, boolean toRight) {
      double angle = pose.getRotation().getRadians();
      double perpAngle = angle + (toRight ? -Math.PI/2 : Math.PI/2); // +90° for left, -90° for right
      
      double offsetX = pose.getX() + offsetDistance * Math.cos(perpAngle);
      double offsetY = pose.getY() + offsetDistance * Math.sin(perpAngle);
      
      return new Pose2d(offsetX, offsetY, pose.getRotation());
    }
    
    /**
     * Gets trough positions with alternating left/right offsets based on position in the hexagon
     * @return Array of offset trough positions
     */
    public static Pose2d[] getTroughPositions() {
      Pose2d[] troughPositions = new Pose2d[POLES.length];
      
      for (int i = 0; i < POLES.length; i++) {
        Pose2d polePose = POLES[i];
        boolean isRightPole = (i % 2 == 0); // Even indices are right poles
        
        // Apply angle offset - rotate away from center
        // right poles get rotated right, left poles get rotated left
        Rotation2d troughRotation = isRightPole ? 
          polePose.getRotation().plus(getTroughAngleOffset()) : 
          polePose.getRotation().minus(getTroughAngleOffset());
        
        // Move inward by the specified distance
        double troughX = polePose.getX() - getTroughOutwardDistance() * Math.cos(troughRotation.getRadians());
        double troughY = polePose.getY() - getTroughOutwardDistance() * Math.sin(troughRotation.getRadians());
        
        // Apply perpendicular offset
        // L3 poles have trough to right, L2 poles have trough to left
        boolean offsetToRight = isRightPole;
        Pose2d baseTroughPose = new Pose2d(troughX, troughY, troughRotation);
        troughPositions[i] = getPerpendicularOffsetPose(baseTroughPose, getTroughOffset(), offsetToRight);
      }
      
      return troughPositions;
    }
  }
}
