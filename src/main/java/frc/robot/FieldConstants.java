package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Constants for points on the field
 * Relative to the blue side origin
 */
public final class FieldConstants {

  public enum WaypointType {
    REEF,
    PROCESSOR,
    CAGE
  }

  // TODO: Adjust for each field type
  public static final double WIDTH = Units.inchesToMeters(317); // TODO: Confirm size 
  public static final double LENGTH = Units.inchesToMeters(690.875); // TODO: Confirm size 

  public static final class WAYPOINTS {
    private WAYPOINTS() {}; 
    // Reef Waypoints, Lettering goes counterclockwise blue, clockwise red
    // TODO: Set real values
    public static final Pose2d PROCESSOR = new Pose2d(3.165, 4.195, Rotation2d.fromDegrees(0));

    public static final double offset = 0;
    public static final double pathOffset = 2.0;

    public static final Pose2d[] POLES = {
      // Temp welded
      new Pose2d(3.165, 4.195, Rotation2d.fromDegrees(0)), // A
      new Pose2d(3.165, 3.860, Rotation2d.fromDegrees(0)), // B
      new Pose2d(3.685, 2.960, Rotation2d.fromDegrees(60)), // C
      new Pose2d(3.970, 2.803, Rotation2d.fromDegrees(60)), // D
      new Pose2d(5.010, 2.800, Rotation2d.fromDegrees(120)), // E
      new Pose2d(5.290, 2.965, Rotation2d.fromDegrees(120)), // F
      new Pose2d(5.810, 3.860, Rotation2d.fromDegrees(180)), // G
      new Pose2d(5.810, 4.185, Rotation2d.fromDegrees(180)), // H
      new Pose2d(5.290, 5.090, Rotation2d.fromDegrees(240)), // I
      new Pose2d(5.000, 5.250, Rotation2d.fromDegrees(240)), // J
      new Pose2d(3.970, 5.250, Rotation2d.fromDegrees(300)), // K
      new Pose2d(3.685, 5.090, Rotation2d.fromDegrees(300)) // L

    };

    private static Pose2d getOffsetPose(Pose2d pose, double offsetDistance) {
      double offsetX = pose.getX() - offsetDistance * Math.cos(pose.getRotation().getRadians());
      double offsetY = pose.getY() - offsetDistance * Math.sin(pose.getRotation().getRadians());
      return new Pose2d(offsetX, offsetY, pose.getRotation());
    }

    public static final Pose2d[] REEF = {
      getOffsetPose(POLES[0], offset),  // A
      getOffsetPose(POLES[1], offset),  // B
      getOffsetPose(POLES[2], offset),  // C
      getOffsetPose(POLES[3], offset),  // D
      getOffsetPose(POLES[4], offset),  // E
      getOffsetPose(POLES[5], offset),  // F
      getOffsetPose(POLES[6], offset),  // G
      getOffsetPose(POLES[7], offset),  // H
      getOffsetPose(POLES[8], offset),  // I
      getOffsetPose(POLES[9], offset),  // J
      getOffsetPose(POLES[10], offset), // K
      getOffsetPose(POLES[11], offset)  // L
    };
  }
}
