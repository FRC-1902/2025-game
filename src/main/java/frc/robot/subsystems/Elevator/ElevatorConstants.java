// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class ElevatorConstants {
    
    public static final class MotorIDs{
        public static final int LEFT_MOTOR_ID = 23;
        public static final int RIGHT_MOTOR_ID = 24;

        public static final int LIMIT_SWITCH_ID = 1;
        public static final int SERVO_ID = 9;
    }

    public static final class PIDConstants{
        public static final double kP = 4.6;
        public static final double kI = 0.0005;
        public static final double kD = 0;
        public static final double kF = 0.0125;  //0.0.125
        public static final double kS = 0.007; // friction
    }

    public static final class CONVERSIONS{
        public static final double TOLERANCE = 0.015; // In meters
        public static final double CONVERSION_FACTOR = 0.0193145; // converts to meters 
        public static final double LOCK_ANGLE = 90; // In degrees
        public static final double UNLOCK_ANGLE = 45.0;
    }

    public enum Position{ 
      // center carriage to floor 9.375 inches, 0.23825 meters, in meters
      L1(Units.inchesToMeters(13.5)), 
      L2(0.58), 
      L3(0.96), 
      CLIMB_UP(.3),
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

    public static final class SimulationConstants{
        public static final class ElevatorSimSetup {
          public static final double GEARING = 16;
          public static final double CARRIAGE_MASS = 0.50893064; // in Kg
          public static final double DRUM_RADIUS = 0.024384; // in metres
          public static final double MIN_HEIGHT = 0; // in metres
          public static final double MAX_HEIGHT = 0.96; // in metres
          public static final double STARTING_HEIGHT = 0; // in metres 
          public static final boolean SIMULATE_GRAVITY = true;
        }

        //TODO: prob wanted to be adjusted and tested further
        public static final class PIDConstants{
          public static final double kP = 11; 
          public static final double kI = 0.0005;
          public static final double kD = 0;
          public static final double kF = 0.0125;  //0.0.125
          public static final double kS = 0.007;
          public static final double kG = 0; // friction
      }

    }
}
