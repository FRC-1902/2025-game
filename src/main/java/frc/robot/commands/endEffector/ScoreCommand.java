// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;
public class ScoreCommand extends Command {
  private final EndEffectorSubsystem endEffectorSubsystem; 

  /**
   * creates command to run end effector until piece is no longer detected (spit on it)
   * @param endEffectorSubsystem
   */
  public ScoreCommand(EndEffectorSubsystem endEffectorSubsystem) {
    this.endEffectorSubsystem = endEffectorSubsystem; 

    addRequirements(endEffectorSubsystem);
  }

  @Override
  public void initialize() {
    endEffectorSubsystem.setSpeed(1.0);
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
