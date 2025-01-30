// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants.WaypointType;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Add your docs here. */
public class AutoDriveFactory {
    private SwerveSubsystem swerve;
    private WaypointType waypoint;
    private PathConstraints constraints;

    public AutoDriveFactory(SwerveSubsystem swerve, WaypointType waypoint) {
        this.swerve = swerve;
        this.waypoint = waypoint;

        constraints = new PathConstraints(
            Constants.Swerve.MAX_SPEED, 
            Constants.Swerve.MAX_ACCELERATION, 
            Constants.Swerve.MAX_ROTATION_SPEED.getRotations(), 
            Constants.Swerve.MAX_ROTATION_SPEED.getRotations()
        );
    }

    public Command driveToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, constraints, 0);
    }

    /**
     * Drives to the waypoint and snaps to it.
     * 
     * @return the command
     */
    public Command pathAndSnapCommand() {
        Pose2d targetPose = swerve.getWaypoint(waypoint);
        return Commands.sequence(
            driveToPose(targetPose),
            new SnapToWaypoint(swerve, targetPose)
        );
    }

}
