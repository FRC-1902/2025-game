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
public class ContinuallySnapToWaypoint extends SnapToWaypoint {
  private final SwerveSubsystem swerve;
  private WaypointType waypointType;

  /** Creates a new SnapToWaypoint. */
  public ContinuallySnapToWaypoint(SwerveSubsystem swerve, Pose2d targetPose) {
    super(swerve, targetPose);
    this.swerve = swerve;
    this.waypointType = waypointType;
  }


  @Override
  public void initialize() {
    super.execute();
  }

  @Override
  public void execute() {
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
