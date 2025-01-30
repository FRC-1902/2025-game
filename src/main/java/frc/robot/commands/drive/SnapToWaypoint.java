// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.WaypointType;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SnapToWaypoint extends Command {
  private final SwerveSubsystem swerve;
  private WaypointType waypointType;
  private Pose2d targetPose;

  /** Creates a new SnapToWaypoint. */
  public SnapToWaypoint(SwerveSubsystem swerve, Pose2d targetPose) {
    this.swerve = swerve;
    this.targetPose = targetPose;

    addRequirements(swerve);
  }


  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

      // Current robot pose
      Pose2d currentPose = swerve.getPose();

      // Simple P-controllers for translation and rotation
      double velocitykP = 1.10; 
      double rotationkP = 1.10;

      Translation2d velocity = targetPose.getTranslation().minus(currentPose.getTranslation()).times(velocitykP);
      Rotation2d rotation = targetPose.getRotation().minus(currentPose.getRotation()).times(rotationkP);

      swerve.drive(velocity, rotation.getRadians(), true); // TODO: Change to radians or degrees
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
          swerve.getPose().getTranslation().getDistance(targetPose.getTranslation());
      double currentRotError = Math.abs(
          swerve.getPose().getRotation().getRadians()
          - targetPose.getRotation().getRadians()
      );

      boolean positionReached = (currentDistance < distanceThreshold);
      boolean rotationReached = (currentRotError < rotationThreshold);

      return positionReached && rotationReached;
  }
}
