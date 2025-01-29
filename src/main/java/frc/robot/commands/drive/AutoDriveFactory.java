// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants.WaypointType;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Add your docs here. */
public class AutoDriveFactory {
    private SwerveSubsystem swerve;
    private WaypointType waypoint;

    public AutoDriveFactory(SwerveSubsystem swerve, WaypointType waypoint) {
        this.swerve = swerve;
        this.waypoint = waypoint;
    }

    public Command pathAndSnapCommand() {
        return Commands.sequence(
            swerve.driveToPose(swerve.getWaypoint(waypoint)), // Turns out there is already a driveToPose method in SwerveSubsystem
            new SnapToWaypoint(swerve, waypoint)
        );
    }

}
