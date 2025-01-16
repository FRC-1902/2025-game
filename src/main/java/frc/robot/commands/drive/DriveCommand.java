// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveCommand extends Command {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY, heading;

    // Values > 1 increase sensitivity
    // Values closer to 1 maintain original sensitivity
    // Values < 1 decrease sensitivity
    private final double ROTATION_SENSITIVITY = 0.5;

    /**
     * Used to drive a swerve robot in full field-centric mode. vX and vY supply
     * translation inputs, where x is
     * torwards/away from alliance wall and y is left/right. headingHorzontal and
     * headingVertical are the Cartesian
     * coordinates from which the robot's angle will be derived— they will be
     * converted to a polar angle, which the robot
     * will rotate to.
     *
     * @param swerve  The swerve drivebase subsystem.
     * @param vX      DoubleSupplier that supplies the x-translation joystick input.
     *                Should be in the range -1 to 1 with
     *                deadband already accounted for. Positive X is away from the
     *                alliance wall.
     * @param vY      DoubleSupplier that supplies the y-translation joystick input.
     *                Should be in the range -1 to 1 with
     *                deadband already accounted for. Positive Y is towards the left
     *                wall when looking through the driver
     *                station glass.
     * @param heading DoubleSupplier that supplies the robot's heading angle.
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

        double allianceMultiplier = -1;

		// Additional alliance-based inversions
		if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
			// Invert x and y velocities for blue alliance
            allianceMultiplier = -1;
		}

        double xVelocity = vX.getAsDouble() * allianceMultiplier;
        double yVelocity = vY.getAsDouble() * allianceMultiplier;

        // Manual rotation control with sensitivity
        double rotationVelocity = heading.getAsDouble();
        rotationVelocity = -MathUtil.applyDeadband(
            rotationVelocity * swerve.getMaximumAngularVelocity(),
            Constants.Controller.RIGHT_X_DEADBAND
        );
        rotationVelocity *= ROTATION_SENSITIVITY;

        // Create field-relative ChassisSpeeds
        ChassisSpeeds fieldRelativeSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity, yVelocity, rotationVelocity, swerve.getHeading()
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
