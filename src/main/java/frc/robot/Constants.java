// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import swervelib.math.Matter;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
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
      private AlgaeIntake() {}; 
      // todo: find id's
      public static final int ROLLER_ID = 11; 
      public static final int PIVOT_ID = 12; 
      public static final int IR_SENSOR_ID = 0; 
      // todo: find p, i, d, g
      public static final double kP = 0; 
      public static final double kI = 0; 
      public static final double kD = 0; 
      public static final double kG = 0;

      public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(1); // TODO: Set tolerance
      public static final Rotation2d MAX_PIVOT = Rotation2d.fromDegrees(90); // TODO: Set MAX
      public static final Rotation2d MIN_PIVOT = Rotation2d.fromDegrees(0);  // TODO: Set MIN
      // todo: find rotations
      public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromRotations(0);
    }

  public static final class FloorIntake{
    private FloorIntake() {}; 
    //todo: set ports
    public static final int ROLLERMOTOR_PORT = 13; 
    public static final int PIVOTMOTOR_PORT = 14;
    public static final int IR_SENSOR_PORT = 15; 
    public static final int ENCODER_PORT = 16; 

    //todo: find p, i, d, g
    public static final double PIVOT_P = 0; 
    public static final double PIVOT_I = 0; 
    public static final double PIVOT_D = 0;
    public static final double PIVOT_G = 0; 

    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(1); // TODO: Set tolerance
    public static final Rotation2d MAX_PIVOT = Rotation2d.fromDegrees(180); // TODO: set MAX
    public static final Rotation2d MIN_PIVOT = Rotation2d.fromDegrees(0);  // TODO: set MIN
    
    //todo: set offset in range from 0 to 1
    public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromDegrees(0);
  } 

  public static final class LED{
    private LED() {}; 
    //todo: set ports and length
    public static final int LED_PORT = 1;
    public static final int LED_LENGTH = 10; 
  }

  public static final class Elevator{
    private Elevator() {}; 
    // todo: find id's
    public static final int LEFT_MOTOR_ID = 17; 
    public static final int RIGHT_MOTOR_ID = 18;  
    // todo: find limit switch/servo ports 
    public static final int LIMIT_SWITCH_PORT = 11; 
    public static final int SERVO_PORT = 10;
    // todo: find p, i, d, f values 
    public static final double kP = 0; 
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0.0; 

    public static final double TOLERANCE = 1; // todo: find tolerance
    public static final double CONVERSION_FACTOR = 0.0095758; // todo; check number, converts to meters 
    public static final Rotation2d LOCK_ANGLE = Rotation2d.fromDegrees(180); // todo: find optimal lock angle
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
      private EndEffector() {}; 
      // todo: set ports 
      public static final int ROLLER_MOTOR_ID = 19; 
      // todo: set channels
      public static final int FRONT_SENSOR_CHANNEL = 20; 
      public static final int BACK_SENSOR_CHANNEL = 21; 
    }
    public static final class Vision {
        // Maximum allowed ambiguity for the cameras
        public static final double MAXIMUM_AMBIGUITY = 0.25; // TODO: Adjust later

        // Camera Configs
        // TODO: Set real values
        public enum Camera {
            ArducamOne(
                "Arducam_1",
                new Rotation3d(Math.toRadians(0), Math.toRadians(-18), Math.toRadians(39.48)),
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
                "Arducam_2",
                new Rotation3d(Math.toRadians(0), Math.toRadians(-18), Math.toRadians(-39.48)), // -39.48
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
                "Arducam_3",
                new Rotation3d(0, Units.degreesToRadians(-145), 0),
                new Translation3d(
                    Units.inchesToMeters(-4.628),
                    Units.inchesToMeters(-10.687),
                    Units.inchesToMeters(16.129)
                ),
                VecBuilder.fill(4, 4, 8),
                VecBuilder.fill(0.5, 0.5, 1)
            );


            public final String name;
            public final Rotation3d rotation;
            public final Translation3d translation;
            public final Matrix<N3, N1> singleTagStdDevs;
            public final Matrix<N3, N1> multiTagStdDevs;

            Camera(
            String name,
            Rotation3d rotation,
            Translation3d translation,
            Matrix<N3, N1> singleTagStdDevs,
            Matrix<N3, N1> multiTagStdDevs) {
                this.name = name;
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