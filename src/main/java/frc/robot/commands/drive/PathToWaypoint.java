// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathToWaypoint extends Command {
  private PathConstraints constraints;
  private Supplier<Pose2d> targetPose;
  private Command pathCommand;
  private SwerveSubsystem swerve;

  /** Creates a new PathToWaypoint. */
  public PathToWaypoint(SwerveSubsystem swerve, Supplier<Pose2d> targetPose) {
    this.targetPose = targetPose;

    constraints = new PathConstraints(
        Constants.Swerve.AUTO_MAX_SPEED, 
        Constants.Swerve.AUTO_MAX_ACCELERATION, 
        Constants.Swerve.AUTO_MAX_ROTATION_SPEED.getRotations(), 
        Constants.Swerve.AUTO_MAX_ROTATION_SPEED.getRotations() // TODO: Change to rotation accel
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathCommand = AutoBuilder.pathfindToPose(targetPose.get(), constraints, 0);
    pathCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogManager.log("SCREAM" + interrupted);

    pathCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: ends prematurely
    return pathCommand.isFinished();
  }
}
