// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FloorIntakeCommand extends Command {
  private final FloorIntake floorIntakeSubsystem; 
  private boolean earlyExit; 

  /** Creates a new FloorIntakeCommand. */
  public FloorIntakeCommand(FloorIntake floorIntakeSubsystem) {
    this.floorIntakeSubsystem = floorIntakeSubsystem; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(floorIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    earlyExit = floorIntakeSubsystem.pieceSensorActive(); 
    if(earlyExit){
      return;
    }
    floorIntakeSubsystem.setSpeed(-1);
    floorIntakeSubsystem.setAngle(Rotation2d.fromDegrees(90)); // todo: find degrees of downwards position
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    floorIntakeSubsystem.setSpeed(0);
    floorIntakeSubsystem.setAngle(Rotation2d.fromDegrees(0)); // todo: find degrees of downwards position
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return earlyExit || floorIntakeSubsystem.pieceSensorActive();
  }
}
