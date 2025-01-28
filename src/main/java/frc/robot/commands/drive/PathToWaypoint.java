// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathToWaypoint extends Command {
  private final SwerveSubsystem swerve;
  private PathConstraints constraints;
  private Pose2d targetPose;
  private Command pathCommand;

  /** Creates a new PathToWaypoint. */
  public PathToWaypoint(SwerveSubsystem swerve) {
    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    constraints = new PathConstraints(
        Constants.Swerve.MAX_SPEED,                   // Max linear velocity (m/s)
        Constants.Swerve.MAX_ACCELERATION,            // Max linear acceleration (m/s^2)
        Constants.Swerve.MAX_ROTATION_SPEED.getRadians(), // Max rotational velocity (rad/s)
        Constants.Swerve.MAX_ROTATION_SPEED.getRadians()  // Max rotational acceleration (rad/s^2)
    );

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPose = swerve.getReefWaypoint();
    pathCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0);
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
        pathCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand != null && pathCommand.isFinished();
  }
}
