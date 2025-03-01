// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PlaceCommand extends Command {
  private final EndEffectorSubsystem endEffectorSubsystem; 

  /** Creates a new EndEffectorCommand. */
  public PlaceCommand(EndEffectorSubsystem endEffectorSubsystem) {
    this.endEffectorSubsystem = endEffectorSubsystem; 

    addRequirements(endEffectorSubsystem);
  }

  @Override
  public void initialize() {
    endEffectorSubsystem.setSpeed(0.35);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    endEffectorSubsystem.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return !endEffectorSubsystem.isFrontPieceSensorActive();
  }
}
