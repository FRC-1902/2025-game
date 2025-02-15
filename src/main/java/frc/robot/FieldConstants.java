package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

  public static final class BLUE {
    private BLUE() {}; 
    // Reef Waypoints, Lettering goes counterclockwise blue, clockwise red
    // TODO: Set real values
    public static final Pose2d[] REEF = {
      new Pose2d(3.250, 4.190, Rotation2d.fromDegrees(0)), // A
      new Pose2d(3.250, 3.860, Rotation2d.fromDegrees(0)), // B
      new Pose2d(3.720, 3.040, Rotation2d.fromDegrees(60)), // C
      new Pose2d(4.020, 2.870, Rotation2d.fromDegrees(60)), // D
      new Pose2d(4.970, 2.870, Rotation2d.fromDegrees(120)), // E
      new Pose2d(5.245, 3.040, Rotation2d.fromDegrees(120)), // F
      new Pose2d(5.725, 3.860, Rotation2d.fromDegrees(180)), // G
      new Pose2d(5.725, 4.190, Rotation2d.fromDegrees(180)), // H
      new Pose2d(5.245, 5.010, Rotation2d.fromDegrees(240)), // I
      new Pose2d(4.970, 5.175, Rotation2d.fromDegrees(240)), // J
      new Pose2d(4.020, 5.175, Rotation2d.fromDegrees(300)), // K
      new Pose2d(3.720, 5.010, Rotation2d.fromDegrees(300)) // L
    };

    public static final Pose2d[] PROCESSOR = {
      new Pose2d(6.000, 0.450, Rotation2d.fromDegrees(270)) // TODO: Set real values
    };

    public static final Pose2d[] CAGE = {
      new Pose2d(8.750, 5.100, Rotation2d.fromDegrees(0)), // TODO: Set real values
      new Pose2d(8.750, 6.200, Rotation2d.fromDegrees(0)), // TODO: Set real values
      new Pose2d(8.750, 7.300, Rotation2d.fromDegrees(0)) // TODO: Set real values
    };
  }

  public static final class RED {
    private RED() {}; 
    public static final Pose2d[] REEF = {
      new Pose2d(3.250, 4.190, Rotation2d.fromDegrees(0)), // A
      new Pose2d(3.250, 3.860, Rotation2d.fromDegrees(0)), // B
      new Pose2d(3.720, 3.040, Rotation2d.fromDegrees(60)), // C
      new Pose2d(4.020, 2.870, Rotation2d.fromDegrees(60)), // D
      new Pose2d(4.970, 2.870, Rotation2d.fromDegrees(120)), // E
      new Pose2d(5.245, 3.040, Rotation2d.fromDegrees(120)), // F
      new Pose2d(5.725, 3.860, Rotation2d.fromDegrees(180)), // G
      new Pose2d(5.725, 4.190, Rotation2d.fromDegrees(180)), // H
      new Pose2d(5.245, 5.010, Rotation2d.fromDegrees(240)), // I
      new Pose2d(4.970, 5.175, Rotation2d.fromDegrees(240)), // J
      new Pose2d(4.020, 5.175, Rotation2d.fromDegrees(300)), // K
      new Pose2d(3.720, 5.010, Rotation2d.fromDegrees(300)) // L
    };
    public static final Pose2d[] PROCESSOR = {
      new Pose2d(3.2, 3.830, Rotation2d.fromDegrees(0)) // TODO: Set real values
    };

    public static final Pose2d[] CAGE = {
      new Pose2d(3.2, 3.830, Rotation2d.fromDegrees(0)), // TODO: Set real values
      new Pose2d(3.2, 3.830, Rotation2d.fromDegrees(0)), // TODO: Set real values
      new Pose2d(3.2, 3.830, Rotation2d.fromDegrees(0)) // TODO: Set real values
    };
  }
}
