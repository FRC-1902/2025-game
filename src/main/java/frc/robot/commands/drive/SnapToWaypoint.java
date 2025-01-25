// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SnapToWaypoint extends Command {
  private final SwerveSubsystem swerve;
  private Translation2d targetTranslation;
  private Rotation2d targetRotation;

  /** Creates a new SnapToWaypoint. */
  public SnapToWaypoint(SwerveSubsystem swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
  }


  @Override
  public void initialize() {
      targetTranslation = swerve.getReefWaypoint().getTranslation();
      targetRotation = swerve.getReefWaypoint().getRotation();
  }

  @Override
  public void execute() {
      // Called every 20ms or so while this command is scheduled

      // Current robot pose
      Translation2d currentTranslation = swerve.getPose().getTranslation();
      Rotation2d currentRotation = swerve.getPose().getRotation();

      // Simple P-controllers for translation and rotation
      double velocitykP = 10.0; 
      double rotationkP = 10.0;

      Translation2d velocity =
          targetTranslation.minus(currentTranslation).times(velocitykP);

      // Convert rotational difference to a Rotation2d, multiply by rotationkP
      Rotation2d rotationError =
          targetRotation.minus(currentRotation).times(rotationkP);

      // Drive with the computed velocity and rotation
      // "rotationError.getRotations()" gives you the rotation in *rotations*, 
      // but presumably you want degrees or radians. Double-check what your swerve.drive requires.
      swerve.drive(velocity, rotationError.getRotations(), true);
  }

  @Override
  public void end(boolean interrupted) {
      // Called once when this command finishes or is interrupted
  }

  @Override
  public boolean isFinished() {
      // Finish when position and orientation are close enough
      double distanceThreshold = 0.01;        // meters
      double rotationThreshold = Math.toRadians(1); // 1 degree in radians

      double currentDistance =
          swerve.getPose().getTranslation().getDistance(targetTranslation);
      double currentRotError = Math.abs(
          swerve.getPose().getRotation().getRadians()
          - targetRotation.getRadians()
      );

      boolean positionReached = (currentDistance < distanceThreshold);
      boolean rotationReached = (currentRotError < rotationThreshold);

      return positionReached && rotationReached;
  }
}
