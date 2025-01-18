// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Elevator.Position;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorCommand extends Command {
  private final ElevatorSubsystem elevator;
  private final Position targetPosition;

  /**
   * Creates a new ElevatorCommand.
   *
   * @param elevator The elevator subsystem to control.
   * @param targetPosition The target position for the elevator.
   */
  public ElevatorCommand(ElevatorSubsystem elevator, Position targetPosition) {
    this.elevator = elevator;
    this.targetPosition = targetPosition;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPosition(targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // The periodic method in the subsystem handles PID control and motor output.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.pidAtSetpoint();
  }
}
