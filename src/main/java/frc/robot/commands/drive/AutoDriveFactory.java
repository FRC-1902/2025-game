// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants.WaypointType;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Add your docs here. */
public class AutoDriveFactory {
    private SwerveSubsystem swerve;

    public AutoDriveFactory(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    /**
     * Drives to the waypoint and snaps to it.
     * 
     * @return the command
     */
    public Command pathAndSnapCommand(WaypointType waypoint) {
        return new SequentialCommandGroup(
            new PathToWaypoint(swerve, () -> swerve.getWaypoint(waypoint)),
            new SnapToWaypoint(swerve, () -> swerve.getWaypoint(waypoint))
        );
    }

}
