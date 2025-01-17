// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveCommand extends Command {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY, heading;

    /**
     * Creates a DriveCommand
     * @param swerve
     * @param vX
     * @param vY
     * @param heading
     */
    public DriveCommand(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.heading = heading;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Apply alliance-based inversions
        var alliance = DriverStation.getAlliance();
        double xVelocity = vX.getAsDouble();
        double yVelocity = vY.getAsDouble();

        // Additional alliance-based inversions
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            // Invert x and y velocities for blue alliance
            xVelocity *= -1;
            yVelocity *= -1;
        }

        double rotationVelocity = heading.getAsDouble();

        // Create field-relative ChassisSpeeds
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity, 
            yVelocity, 
            rotationVelocity, 
            swerve.getHeading()
        );

        // Drive using field-relative speeds
        swerve.drive(fieldRelativeSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
    } 

    @Override
    public boolean isFinished() {
        return false;
    }
}
