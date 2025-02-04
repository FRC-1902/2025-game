// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeOuttakeCommand extends Command {
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem; 
  
  /** Creates a new AlgaeIntakeCommand. */
  public AlgaeOuttakeCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeIntakeSubsystem.setSpeed(1); // todo: find actual speed 
    algaeIntakeSubsystem.setAngle(Rotation2d.fromDegrees(-45)); // todo: find actual downward angle 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntakeSubsystem.setAngle(Constants.AlgaeIntake.DEFAULT_ANGLE); // todo: find upward angle 
    algaeIntakeSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
  }
}
