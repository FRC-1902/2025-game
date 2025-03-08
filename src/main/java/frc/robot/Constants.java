package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
  public static final double BATTERY_VOLTAGE_CRITICAL = 11.5; // Volts TODO: Adjust later
  public static final double BATTERY_VOLTAGE_WARNING = 12.0; // Volts TODO: Adjust later

  public static final class Controller{
    private Controller() {}
    // Controller Ports
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int MANIP_CONTROLLER_PORT = 1;

    // Joystick Deadband
    public static final double RIGHT_Y_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class Swerve {
    private Swerve() {}
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag TDO: Adjust later

    // Speeds
    public static final double MAX_SPEED = Units.feetToMeters(16.6); // ft/s
    public static final double MAX_ACCELERATION = Units.feetToMeters(12.0); // ft/s^2
    public static final Rotation2d MAX_ROTATION_SPEED = Rotation2d.fromRadians(10);

    // Auto Speeds
    public static final double AUTO_MAX_SPEED = Units.feetToMeters(5); // ft/s TODO: Adjust later
    public static final double AUTO_MAX_ACCELERATION = Units.feetToMeters(5); // ft/s^2 TODO: Adjust later
    public static final Rotation2d AUTO_MAX_ROTATION_SPEED = Rotation2d.fromRadians(3); // TODO: Adjust later

    // todo: figure out magic ##
    public static final double OBJECT_TURN_KP = 4; // describes how much gas to give the robot to turn
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

    public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromDegrees(57.3); 

    public static final Rotation2d DEFAULT_ANGLE = Rotation2d.fromDegrees(90);
  }

  public static final class FloorIntake{
    private FloorIntake() {}
    public static final int ROLLER_MOTOR_ID = 20; 
    public static final int PIVOT_MOTOR_ID = 3;
    public static final int PIECE_SENSOR_ID = 8; 
    public static final int PIVOT_ENCODER_ID = 20; 

    public static final double PIVOT_P = 0.0125; 
    public static final double PIVOT_I = 0.0035; 
    public static final double PIVOT_D = 0.00005; 
    public static final double PIVOT_G = 0.022; //.022 

    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(3); 
    public static final Rotation2d MAX_PIVOT = Rotation2d.fromDegrees(350); 
    public static final Rotation2d MIN_PIVOT = Rotation2d.fromDegrees(190); 
  
    public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromDegrees(349.53);

    public static final double HP_ANGLE = 120.0;  // TODO: Find optimal angle
    public static final double FLOOR_ANGLE = 178.0;
    public static final double ELEVATOR_ANGLE = 80.0;
    public static final double CLIMB_ANGLE = 90.0;
    public static final double DEFAULT_ANGLE = 5;
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
    public static final double kS = 0.0069;

    public static final double TOLERANCE = 0.015; // In meters
    public static final double CONVERSION_FACTOR = 0.0193145; // converts to meters 
    public static final double LOCK_ANGLE = 166; // In degrees
    public static final double UNLOCK_ANGLE = 150.0;  

    public enum Position{ 
      // center carriage to floor 9.375 inches, 0.23825 meters, in meters
      L1(0.34), 
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
    // Maximum allowed ambiguity for the cameras
    public static final double MAXIMUM_AMBIGUITY = 0.25; // TODO: Adjust later

    public static final class ObjectDetection {
      public static final String CAMERA_NAME = "colorCam";
      public static final double CONFIDENCE_THRESHOLD = 0.5; // TODO: change later
      public static final Rotation2d ANGLE = Rotation2d.fromDegrees(50); // TODO: Change to real value
      public static final double HEIGHT = 29; // TODO: Change to real value
      public static final Rotation2d HORIZONTAL_FOV = Rotation2d.fromDegrees(65); // TODO: Change to real value
      public static final Rotation2d VERTICAL_FOV = Rotation2d.fromDegrees(53); // TODO: Change to real value
      public static final double HORIZONTAL_RES = 1920;
      public static final double VERTICAL_RES = 1080;
      public static final Translation3d CAMERA_TRANSLATION = new Translation3d(0, 0, 0);
      public static final Rotation3d CAMERA_ROTATION = new Rotation3d(0, 0, 0);
    }

    // Camera Configs
    public enum Camera {
      CAMERA_ONE(
        "arducamOne",
        new Rotation3d(Math.toRadians(0), Math.toRadians(-22), Math.toRadians(38)),
        new Translation3d(
          // Units.inchesToMeters(11.233), 
          // Units.inchesToMeters(9.691),
          // Units.inchesToMeters(8.513920)
          Units.inchesToMeters(9.883),
          Units.inchesToMeters(-11.103),
          Units.inchesToMeters(8.069)
        ),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1)
      ),

      CAMERA_TWO(
        "arducamFour",
        new Rotation3d(Math.toRadians(0), Math.toRadians(-22), Math.toRadians(-38)),
        new Translation3d(
          // Units.inchesToMeters(-11.233), 
          // Units.inchesToMeters(9.691), 
          // Units.inchesToMeters(8.513920)
          Units.inchesToMeters(9.883),
          Units.inchesToMeters(11.103),
          Units.inchesToMeters(8.069)
        ),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1)
      ),

      CAMERA_THREE(
        "arducamThree",
        new Rotation3d(0, Units.degreesToRadians(-145), 0),
        new Translation3d(
            Units.inchesToMeters(-4.628),
            Units.inchesToMeters(-10.687),
            Units.inchesToMeters(16.129)
        ),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1)
      );

      public final String camName;
      public final Rotation3d rotation;
      public final Translation3d translation;
      public final Matrix<N3, N1> singleTagStdDevs;
      public final Matrix<N3, N1> multiTagStdDevs;

      Camera(
      String camName,
      Rotation3d rotation,
      Translation3d translation,
      Matrix<N3, N1> singleTagStdDevs,
      Matrix<N3, N1> multiTagStdDevs) {
        this.camName = camName;
        this.rotation = rotation;
        this.translation = translation;
        this.singleTagStdDevs = singleTagStdDevs;
        this.multiTagStdDevs = multiTagStdDevs;
      }
    }
    public static final Pose3d[] CAMERA_POSITIONS = {
      new Pose3d(Camera.CAMERA_ONE.translation, Camera.CAMERA_ONE.rotation),
      new Pose3d(Camera.CAMERA_TWO.translation, Camera.CAMERA_TWO.rotation),
      new Pose3d(Camera.CAMERA_THREE.translation, Camera.CAMERA_THREE.rotation),
    };
  }
}