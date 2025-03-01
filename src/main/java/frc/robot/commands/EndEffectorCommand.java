// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;
import java.util.function.BooleanSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorCommand extends Command {
  
  private final EndEffectorSubsystem endEffectorSubsystem;
  private double speed;
  private BooleanSupplier conditional;
  
  /**
   * 
   * @param endEffectorSubsystem
   * @param speed
   * @param conditional
   * sets EE to a speed based on some condition. Used mainly to adjust indexing when piece is sent from floor intake.
   */
  /** Creates a new EndEffectorCommand. */
  public EndEffectorCommand(EndEffectorSubsystem endEffectorSubsystem, double speed, BooleanSupplier conditional) {
    this.endEffectorSubsystem = endEffectorSubsystem;
    this.speed = speed;
    this.conditional = conditional;

    addRequirements(endEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffectorSubsystem.setSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffectorSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return conditional.getAsBoolean();
  }
}
