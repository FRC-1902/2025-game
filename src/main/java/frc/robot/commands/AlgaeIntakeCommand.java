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
    earlyExit = algaeIntakeSubsystem.isPieceSensorActive(); // returns if algae is detected or not
    if(earlyExit) {
      return;
    }
    algaeIntakeSubsystem.setSpeed(.9);
    algaeIntakeSubsystem.setAngle(Rotation2d.fromDegrees(20)); 
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      algaeIntakeSubsystem.setAngle(Rotation2d.fromDegrees(75));
      algaeIntakeSubsystem.setSpeed(0.9);
    }
    else{
      algaeIntakeSubsystem.setAngle(Constants.AlgaeIntake.DEFAULT_ANGLE); 
      algaeIntakeSubsystem.setSpeed(0);
    }
  }

  @Override
  public boolean isFinished() {
    return earlyExit || algaeIntakeSubsystem.isPieceSensorActive(); 
  }
}
