// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class Controller{
    // Controller Ports
    public static final int DRIVE_CONTROLLER_PORT = 0; // TODO: Set port
    public static final int MANIP_CONTROLLER_PORT = 1; // TODO: Set port

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class LED{
    private LED() {}; 
    //todo: set ports and length
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 0; 
  }

  public static final class Elevator{
    private Elevator() {}; 
    // todo: find id's
    public static final int LEFT_MOTOR_ID = 0; 
    public static final int RIGHT_MOTOR_ID = 0;  
    // todo: find limit switch/servo ports 
    public static final int LIMIT_SWITCH_PORT = 0; 
    public static final int SERVO_PORT = 0;
    // todo: find p, i, d, f values 
    public static final double kP = 0; 
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0.0; 

    public static final double TOLERANCE = 0; // todo: find tolerance
    public static final double CONVERSION_FACTOR = 0.0095758; // todo; check number
    public static final Rotation2d LOCK_ANGLE = Rotation2d.fromDegrees(69); // todo: find optimal lock angle
    public static final Rotation2d UNLOCK_ANGLE = Rotation2d.fromDegrees(420); // todo: find optimal unlocked angle

    public enum Position{ 
      //todo: set height setpoints in meters
      L1(0), L2(0), L3(0), CLIMB(0), MIN(0), MAX(0); 
      private final double height;

      Position(double height){
        this.height = height;
      }

      public double getHeight(){
        return height;
      }
    }
  }
}
