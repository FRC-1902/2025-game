// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public final class AlgaeConstants {

    public static final class MotorIDs {
        public static final int ROLLER_MOTOR_ID = 19;
        public static final int PIVOT_MOTOR_ID = 12;
        public static final int PIECE_SENSOR_ID = 5;
    }

    public static final class PID {
        public static final double kP = 0.01;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0.0;
    }

    public static final class Positions {
        public static final Rotation2d MAX_PIVOT = Rotation2d.fromDegrees(100);
        public static final Rotation2d MIN_PIVOT = Rotation2d.fromDegrees(20);
        public static final Rotation2d DEFAULT_ANGLE = Rotation2d.fromDegrees(90);
    }

    public static final class Offsets {
        public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(5);
        public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromDegrees(51.5);
    }

    public static final class Simulation{
        public static final double GEARING = 50;  
        public static final double MOMENT = 0.3; 
        public static final double ARM_LENGTH = 0.2286; 
        public static final boolean SIMULATE_GRAVITY = true;
    }

}
