// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.DetectionSubsystem;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToObject extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final DetectionSubsystem detectionSubsystem;
  private double start;

  /** Creates a new DriveToObject. */
  public DriveToObject(SwerveSubsystem swerveSubsystem, DetectionSubsystem detectionSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.detectionSubsystem = detectionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = Timer.getFPGATimestamp();
    DataLogManager.log("Start Drive Aligne");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.drive(new Translation2d(-0.7, 0), 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
    DataLogManager.log("End Drive Aligne");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - start >= 7); //!detectionSubsystem.isTargetVisible()
  }
}