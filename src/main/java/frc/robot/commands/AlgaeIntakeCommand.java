package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeIntakeCommand extends Command {
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem; 
  private boolean earlyExit; 
  
  /** Creates a new AlgaeIntakeCommand. */
  public AlgaeIntakeCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;  

    addRequirements(algaeIntakeSubsystem);
  }

  @Override
  public void initialize() {
    //earlyExit = algaeIntakeSubsystem.isAlgaeDetected(); // returns if algae is detected or not
    if(earlyExit) {
      return;
    }
    algaeIntakeSubsystem.setSpeed(-0.84); // todo: find actual speed 
    algaeIntakeSubsystem.setAngle(Rotation2d.fromDegrees(45)); // todo: find actual downward angle 
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    algaeIntakeSubsystem.setAngle(Constants.AlgaeIntake.DEFAULT_ANGLE); // todo: find upward angle 
    algaeIntakeSubsystem.setSpeed(-0.36);
  }

  @Override
  public boolean isFinished() {
    return false;//earlyExit || algaeIntakeSubsystem.isAlgaeDetected(); 
  }
}
