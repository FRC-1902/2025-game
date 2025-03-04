package frc.robot.commands;

import org.ejml.equation.Sequence;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
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
    new SequentialCommandGroup(
      new InstantCommand(() -> algaeIntakeSubsystem.setAngle(Rotation2d.fromDegrees(50))),
      new InstantCommand(() -> algaeIntakeSubsystem.setSpeed(-0.4)) // todo: find actual speed 
    );
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    algaeIntakeSubsystem.setAngle(Constants.AlgaeIntake.DEFAULT_ANGLE); // todo: find upward angle 
    algaeIntakeSubsystem.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}
