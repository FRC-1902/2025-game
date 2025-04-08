package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

  // Robot Mode
  public enum RobotMode {
    REAL, // Physical robot hardware
    SIM, // Simulation mode
    REPLAY // Replay mode for log analysis
  }
  public static final RobotMode simMode = RobotMode.SIM;
  public static final RobotMode currentMode = RobotBase.isReal() ? RobotMode.REAL : simMode;
  
  // Battery Voltage Warnings
  public static final double BATTERY_VOLTAGE_CRITICAL = 11.5;
  public static final double BATTERY_VOLTAGE_WARNING = 12.0;

  public static final int PDH_ID = 22; // TODO: Find can ID

  public static final class Controller{
    private Controller() {}
    // Controller Ports
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int MANIP_CONTROLLER_PORT = 1;

    // Joystick Deadband
    public static final double RIGHT_Y_DEADBAND = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.05;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class Swerve {
    private Swerve() {}
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

    // Speeds
    public static final double MAX_SPEED = Units.feetToMeters(16.6); // ft/s
    public static final double MAX_ACCELERATION = Units.feetToMeters(12.0); // ft/s^2
    public static final Rotation2d MAX_ROTATION_SPEED = Rotation2d.fromRadians(10);

    // Auto Speeds
    public static final double AUTO_MAX_SPEED = 3; // m/s
    public static final double AUTO_MAX_ACCELERATION = 2; // m/s^2
    public static final Rotation2d AUTO_MAX_ROTATION_SPEED = Rotation2d.fromRadians(4); 
  }

  public static final class AlgaeIntake{
    private AlgaeIntake() {}

    public static final int ROLLER_MOTOR_ID = 19; 
    public static final int PIVOT_MOTOR_ID = 12; 
    public static final int PIECE_SENSOR_ID = 5; 

    public static final double kP = 0.0125;
    public static final double kI = 0; 
    public static final double kD = 0; 
    public static final double kG = 0.01;

    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(3); 
    public static final Rotation2d MAX_PIVOT = Rotation2d.fromDegrees(100); 
    public static final Rotation2d MIN_PIVOT = Rotation2d.fromDegrees(20); 

    public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromDegrees(51.5); 

    public static final Rotation2d DEFAULT_ANGLE = Rotation2d.fromDegrees(90);
  }

  public static final class FloorIntake{
    private FloorIntake() {}
    public static final int ROLLER_MOTOR_ID = 20; 
    public static final int PIVOT_MOTOR_ID = 3;
    public static final int PIECE_SENSOR_ID = 8; 
    public static final int PIVOT_ENCODER_ID = 20; 

    public static final double PIVOT_P = 0.013;// 0.0125; 
    public static final double PIVOT_I = 0.003;// 0.0035; 
    public static final double PIVOT_D = 0.0002;// 0.00005; 
    public static final double PIVOT_G = 0.022;// 0.022;
    public static final double PIVOT_F = 0.03;// 0.022;

    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(3); 
    public static final Rotation2d MAX_PIVOT = Rotation2d.fromDegrees(350); 
    public static final Rotation2d MIN_PIVOT = Rotation2d.fromDegrees(190); 
  
    public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromDegrees(350.5);

    public static final double HP_ANGLE = 120.0;  // TODO: Find optimal angle
    public static final double FLOOR_ANGLE = 180.0;
    public static final double ELEVATOR_ANGLE = 70.0;
    public static final double CLIMB_ANGLE = 90.0;
    public static final double DEFAULT_ANGLE = 3.5;
  } 

  public static final class LED{
    private LED() {}
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 71; 
  }

  public static final class Elevator{
    private Elevator() {}
    public static final int LEFT_MOTOR_ID = 23; 
    public static final int RIGHT_MOTOR_ID = 24;  

    public static final int LIMIT_SWITCH_ID = 1; 
    public static final int SERVO_ID = 9;

    public static final double kP = 4.6;
    public static final double kI = 0.0005;
    public static final double kD = 0;
    public static final double kF = 0.0125;  //0.0.125
    public static final double kS = 0.007; // friction

    public static final double TOLERANCE = 0.015; // In meters
    public static final double CONVERSION_FACTOR = 0.0193145; // converts to meters 
    public static final double LOCK_ANGLE = 90; // In degrees
    public static final double UNLOCK_ANGLE = 50.0;  

    public enum Position{ 
      // center carriage to floor 9.375 inches, 0.23825 meters, in meters
      L1(0.35), 
      L2(0.58), 
      L3(0.96), 
      CLIMB_UP(.3), // TODO: Set height
      CLIMB_DOWN(0), 
      MIN(0), 
      MAX(0.96),
      HOME(0),
      HOLD(0);

      private final double height;

      Position(double height){
        this.height = height;
      }

      public double getHeight(){
        return height;
      }
    }
  }

  public static final class EndEffector{
    private EndEffector() {}
    public static final int ROLLER_MOTOR_ID = 13;

    public static final int FRONT_PIECE_SENSOR_ID = 7; 
    public static final int BACK_PIECE_SENSOR_ID = 4; 
  }

  public static final class Vision {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Left Camera
    public static String CAMERA_ONE = "arducamOne";
    public static Transform3d CAMERA_ONE_POS = new Transform3d(
      new Translation3d( // X (red), Y (green), Z (height)
        Units.inchesToMeters(10.),
        Units.inchesToMeters(-12),
        Units.inchesToMeters(8.563)
      ),
      new Rotation3d(
        Math.toRadians(0), 
        Math.toRadians(-20), 
        Math.toRadians(42)
      )
    );

    // Right Camera
    public static String CAMERA_TWO = "arducamFour";
    public static Transform3d CAMERA_TWO_POS = new Transform3d(
      new Translation3d( // X (red), Y (green), Z (height)
        Units.inchesToMeters(10.),
        Units.inchesToMeters(12),
        Units.inchesToMeters(8.563)
      ),
      new Rotation3d(
        Math.toRadians(0), 
        Math.toRadians(-20), 
        Math.toRadians(-42)
      )
    );

    // Back Camera
    public static String CAMERA_THREE = "arducamThree";
    public static Transform3d CAMERA_THREE_POS = new Transform3d(
      new Translation3d( // X (red), Y (green), Z (height)
        Units.inchesToMeters(-9.537),
        Units.inchesToMeters(-10.806),
        Units.inchesToMeters(8.525)
      ),
      new Rotation3d(
        Math.toRadians(0), 
        Math.toRadians(-20), 
        Math.toRadians(-190)
      )
    );

    // Object Detection Camera
    public static final class CAMERA_OBJECT {
      public static final String CAMERA_NAME = "colorCam";

      public static final double CONFIDENCE_THRESHOLD = 0.5; // TODO: change later

      public static final Rotation2d HORIZONTAL_FOV = Rotation2d.fromDegrees(69.83); // TODO: Change to real value
      public static final Rotation2d VERTICAL_FOV = Rotation2d.fromDegrees(47.16); // TODO: Change to real value
      public static final double HORIZONTAL_RES = 1280;
      public static final double VERTICAL_RES = 800 ;

      public static final Transform3d CAMERA_OBJECT_POS = new Transform3d(
        new Translation3d(
          Units.inchesToMeters(-4.879), // X camera offset
          0.0,                      // Y (centered)
          Units.inchesToMeters(28.5)  // Z (28 inches up) 30.1
        ),
        new Rotation3d(
          0.0,                      //  Roll (no tilt side to side)
          Math.toRadians(65),      // Pitch (30° downward - adjust as needed)
          Math.toRadians(180)       // Yaw (facing backward based on your current CAMERA_POSE)
        )
      );
    }

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0 (Front Camera)
        1.0, // Camera 1 (Back Camera)
        1.0, // Camera 2 (Front Left Camera)
        1.0 // Camera 3 (Front Right Camera)
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
  }
}