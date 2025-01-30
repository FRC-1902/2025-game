// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ContinuallySnapToWaypoint extends SnapToWaypoint {

  /** Creates a new SnapToWaypoint. */
  public ContinuallySnapToWaypoint(SwerveSubsystem swerve, Pose2d targetPose) {
    super(swerve, targetPose);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
