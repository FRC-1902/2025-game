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
import swervelib.math.Matter;

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
    public static final int DRIVE_CONTROLLER_PORT = 0; // TODO: Set port
    public static final int MANIP_CONTROLLER_PORT = 1; // TODO: Set port

    // Joystick Deadband
    public static final double RIGHT_Y_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class Swerve {
    private Swerve() {}
    public static final double ROBOT_MASS = Units.lbsToKilograms(100.000); // kg Adjusted value
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS); // TODO: Adjust later
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag TDO: Adjust later

    // Speeds
    public static final double MAX_SPEED = Units.feetToMeters(16.6); // m/s TODO: Adjust later
    public static final double MAX_ACCELERATION = Units.feetToMeters(34.44882); // m/s^2 TODO: Adjust later
    public static final Rotation2d MAX_ROTATION_SPEED = Rotation2d.fromRadians(10); // m/s TODO: Adjust later

    // Auto Speeds
    public static final double AUTO_MAX_SPEED = Units.feetToMeters(.5); // m/s TODO: Adjust later
    public static final double AUTO_MAX_ACCELERATION = Units.feetToMeters(.7); // m/s^2 TODO: Adjust later
    public static final Rotation2d AUTO_MAX_ROTATION_SPEED = Rotation2d.fromRadians(1); // m/s TODO: Adjust later
  }

  public static final class AlgaeIntake{
    private AlgaeIntake() {}
    // todo: find id's
    public static final int ROLLER_MOTOR_ID = 11; 
    public static final int PIVOT_MOTOR_ID = 5; 
    public static final int IR_SENSOR_ID = 20; 
    // todo: find p, i, d, g
    public static final double kP = 0; 
    public static final double kI = 0; 
    public static final double kD = 0; 
    public static final double kG = 0;
    // todo: find tolerances
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(0); 
    public static final Rotation2d MAX_PIVOT = Rotation2d.fromDegrees(0); 
    public static final Rotation2d MIN_PIVOT = Rotation2d.fromDegrees(0); 
    // todo: find rotations
    public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromRotations(0);

    public static final Rotation2d DEFAULT_ANGLE = Rotation2d.fromDegrees(90);
  }

  public static final class FloorIntake{
    private FloorIntake() {}
    //todo: set ports
    public static final int ROLLER_MOTOR_ID = 6; 
    public static final int PIVOT_MOTOR_ID = 7;
    public static final int IR_SENSOR_ID = 19; 
    public static final int PIVOT_ENCODER_ID = 0; 

    //todo: find p, i, d, g
    public static final double PIVOT_P = 0; 
    public static final double PIVOT_I = 0; 
    public static final double PIVOT_D = 0;
    public static final double PIVOT_G = 0; 

    //todo: set tolerance
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(0); 
    public static final Rotation2d MAX_PIVOT = Rotation2d.fromDegrees(0); 
    public static final Rotation2d MIN_PIVOT = Rotation2d.fromDegrees(0); 
    
    //todo: set offset in range from 0 to 1
    public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromDegrees(0);
  } 

  public static final class LED{
    private LED() {}
    //todo: set ports and length
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 156; 
  }

  public static final class Elevator{
    private Elevator() {}
    // todo: find id's
    public static final int LEFT_MOTOR_ID = 20; 
    public static final int RIGHT_MOTOR_ID = 14;  
    // todo: find limit switch/servo ports 
    public static final int LIMIT_SWITCH_ID = 18; 
    public static final int SERVO_ID = 17;
    // todo: find p, i, d, f values 
    public static final double kP = 0; 
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0.0; 

    public static final double TOLERANCE = 0; // todo: find tolerance
    public static final double CONVERSION_FACTOR = 0.0095758; // todo; check number, converts to meters 
    public static final Rotation2d LOCK_ANGLE = Rotation2d.fromDegrees(0); // todo: find optimal lock angle
    public static final Rotation2d UNLOCK_ANGLE = Rotation2d.fromDegrees(0); // todo: find optimal unlocked angle

    public enum Position{ 
      //todo: set height setpoints in meters
      L1(0), L2(0), L3(0), CLIMB_UP(0), CLIMB_DOWN(0), MIN(0), MAX(0); 
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
    // todo: set ids 
    public static final int ROLLER_MOTOR_ID = 4;
    // todo: set ids
    public static final int FRONT_IR_SENSOR_ID = 16; 
    public static final int BACK_IR_SENSOR_ID = 15; 
  }

  public static final class Vision {
    // Maximum allowed ambiguity for the cameras
    public static final double MAXIMUM_AMBIGUITY = 0.25; // TODO: Adjust later

    // Camera Configs
    // TODO: Set real values
    public enum Camera {
      ArducamOne(
        "arducamOne",
        new Rotation3d(Math.toRadians(0), Math.toRadians(-22.75), Math.toRadians(40)),
        new Translation3d(
          // Units.inchesToMeters(11.233), 
          // Units.inchesToMeters(9.691),
          // Units.inchesToMeters(8.513920)
          Units.inchesToMeters(9.691),
          Units.inchesToMeters(-11.233),
          Units.inchesToMeters(8.513920)
        ),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1)
      ),

      ArducamTwo(
        "arducamTwo",
        new Rotation3d(Math.toRadians(0), Math.toRadians(-22.75), Math.toRadians(-40)),
        new Translation3d(
          // Units.inchesToMeters(-11.233), 
          // Units.inchesToMeters(9.691), 
          // Units.inchesToMeters(8.513920)
          Units.inchesToMeters(9.691),
          Units.inchesToMeters(11.233),
          Units.inchesToMeters(8.513920)
        ),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1)
      ),

      ArducamThree(
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
      new Pose3d(Camera.ArducamOne.translation, Camera.ArducamOne.rotation),
      new Pose3d(Camera.ArducamTwo.translation, Camera.ArducamTwo.rotation),
      new Pose3d(Camera.ArducamThree.translation, Camera.ArducamThree.rotation),
    };
  }
}