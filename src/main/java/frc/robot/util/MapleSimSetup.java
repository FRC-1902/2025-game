// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class MapleSimSetup {

    public static final Pose2d redHPCoralPose = new Pose2d(16.17, 1.33, new Rotation2d());
    public static final Pose2d blueHPCoralPose = FlippingUtil.flipFieldPose(redHPCoralPose);
    public static final double CORAL_LENGTH = 0.3; // in metres 

     public static void spawnHumanPlayerCoral(boolean blue) {
        if (!RobotContainer.MAPLESIM) {
            return;
        }

        for (int i = 0; i < 2; i++) {
            Pose2d coralPose = blue ? blueHPCoralPose : redHPCoralPose;
            
            if (i == 1) {
                coralPose = coralPose.transformBy(new Transform2d(0, 5, new Rotation2d()));
            }

            // generate a random physical offset between -0.3 and 0.3 meters and a random rotation
            double xOffset = randomNumberPlusMinus(0.3);
            double yOffset = randomNumberPlusMinus(0.3);
            double rotationOffset = Math.random() * 360;
            Transform2d randomTransform = new Transform2d(xOffset, yOffset, Rotation2d.fromDegrees(rotationOffset));

            spawnCoral(coralPose.transformBy(randomTransform));
        }
    }

    public static void spawnCoral(Pose2d pose) {
        SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(pose));
    }

    private static double randomNumberPlusMinus(double range) {
        return Math.random() * (range * 2) - range;
    }

}
