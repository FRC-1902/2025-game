// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffector;
import frc.robot.subsystems.EndEffectorSubsystem;
public class ScoreCommand extends Command {
  private final EndEffectorSubsystem endEffectorSubsystem; 

  /**
   * creates command to run end effector until piece is no longer detected (spit on it)
   * @param endEffectorSubsystem
   */
  public ScoreCommand(EndEffectorSubsystem endEffectorSubsystem) {
    this.endEffectorSubsystem = endEffectorSubsystem; 
    SmartDashboard.putNumber("EndEffector/Roller Speed", 0.4); // TODO: remove, here for tuning during match
    addRequirements(endEffectorSubsystem);
  }

  @Override
  public void initialize() {
    double speed = SmartDashboard.getNumber("EndEffector/Roller Speed", 0.4); // TODO: remove, here for tuning during match
    endEffectorSubsystem.setSpeed(speed);
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
