package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeOuttakeCommand extends Command {
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem; 
  
  /** Creates a new AlgaeIntakeCommand. */
  public AlgaeOuttakeCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;  

    addRequirements(algaeIntakeSubsystem);
  }

  @Override
  public void initialize() {
    algaeIntakeSubsystem.setSpeed(-0.2); // todo: find actual speed 
    algaeIntakeSubsystem.setAngle(Rotation2d.fromDegrees(45)); // todo: find actual downward angle 
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    algaeIntakeSubsystem.setAngle(Rotation2d.fromDegrees(0)); // todo: find upward angle 
    algaeIntakeSubsystem.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}
