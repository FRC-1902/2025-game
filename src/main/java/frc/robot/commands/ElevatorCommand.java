package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Elevator.Position;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;


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

  @Override
  public void initialize() {
    elevator.setPosition(targetPosition);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      ControllerSubsystem.getInstance().vibrate(ControllerName.MANIP, 100, 1);
    }
  }

  @Override
  public boolean isFinished() {
    return elevator.pidAtSetpoint();
  }
}
