package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;
import java.util.function.BooleanSupplier;

public class EndEffectorCommand extends Command {
  
  private final EndEffectorSubsystem endEffectorSubsystem;
  private double speed;
  private BooleanSupplier conditional;
  
  /**
   * Runs the end effector at a given speed until a condition is met.
   * @param endEffectorSubsystem
   * @param speed
   * @param conditional
   */
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
