// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FloorIntake;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public final class FloorConstants {

    public static final class MotorIDs {
        public static final int ROLLER_MOTOR_ID = 20;
        public static final int PIVOT_MOTOR_ID = 3;
        public static final int PIECE_SENSOR_ID = 8;
        public static final int PIVOT_ENCODER_ID = 20;
    }

    public static final class PID {
        public static final double kP = 0.001;// 0.0125; 
        public static final double kI = 0.00075;// 0.0035; 
        public static final double kD = 0.0000;// 0.00005; 
        public static final double kG = 0.15;// 0.022;
        public static final double kF = 0.00;// 0.022;
    }

    public static final class Positions {
        public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(3);
        public static final Rotation2d MAX_PIVOT = Rotation2d.fromDegrees(177);
        public static final Rotation2d MIN_PIVOT = Rotation2d.fromDegrees(0);

        public static final double FLOOR_ANGLE = 180.0;
        public static final double ELEVATOR_ANGLE = 70.0;
        public static final double CLIMB_ANGLE = 90.0;
        public static final Rotation2d DEFAULT_ANGLE = Rotation2d.fromDegrees(3.5);

        public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromDegrees(0);
    }

    public static final class Simulation {
        public static final double GEARING = 50; 
        public static final double MOMENT = 0.303; 
        public static final double ARM_LENGTH = 0.22; 
        public static final boolean SIMULATE_GRAVITY = true; 
        public static final class PID{
            public static final double kP = 0.001;// 0.0125; 
            public static final double kI = 0.00075;// 0.0035; 
            public static final double kD = 0.0000;// 0.00005; 
            public static final double kG = 0.15;// 0.022;
            public static final double kF = 0.00;// 0.022;
        }
    }
}