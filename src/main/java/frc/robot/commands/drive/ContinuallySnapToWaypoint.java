// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ContinuallySnapToWaypoint extends SnapToWaypoint {

  /** Creates a new SnapToWaypoint. */
  public ContinuallySnapToWaypoint(SwerveSubsystem swerve, Supplier<Pose2d> targetPoseSupplier) {
    super(swerve, targetPoseSupplier);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
