// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
public class ContinuousConditionalCommand extends Command {
  Command incomingCommand, baseCommand;
  BooleanSupplier supplier;
  private boolean flag;
  /** Creates a new ContinuousConditionalCommand. */
  public ContinuousConditionalCommand(Command baseCommand, Command incomingCommand, BooleanSupplier supplier) {
    this.supplier = supplier;
    this.incomingCommand = incomingCommand;
    this.baseCommand = baseCommand;
    flag = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("Starting ContinuousConditionalCommand");
    baseCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!flag && supplier.getAsBoolean()){
      baseCommand.cancel();
      incomingCommand.schedule();
      DataLogManager.log("Switching to incoming command");
      flag = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    baseCommand.cancel();
    incomingCommand.cancel();
    DataLogManager.log("Ending ContinuousConditionalCommand");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(flag){
      return incomingCommand.isFinished();
    }
    return baseCommand.isFinished();
  }
}
