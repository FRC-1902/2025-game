// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Add your docs here. */
public class AutoDrive {
    private SwerveSubsystem swerve;

    public AutoDrive(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    public Command pathAndSnapCommand() {
        return Commands.sequence(
            new PathToWaypoint(swerve),
            new SnapToWaypoint(swerve)
        );
    }

}
