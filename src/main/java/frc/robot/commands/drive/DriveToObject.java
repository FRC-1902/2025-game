// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
public class DriveToObject extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final FloorIntakeSubsystem floorIntakeSubsystem;

  /**
   * Drives backwards until piece intook by Floor Intake
   * @param swerveSubsystem
   * @param floorIntakeSubsystem
   */
  public DriveToObject(SwerveSubsystem swerveSubsystem, FloorIntakeSubsystem floorIntakeSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.floorIntakeSubsystem = floorIntakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.drive(new Translation2d(-1, 0), 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return floorIntakeSubsystem.pieceSensorActive();
  }
}