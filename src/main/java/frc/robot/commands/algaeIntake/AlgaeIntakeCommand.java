package frc.robot.commands.algaeIntake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeIntakeCommand extends Command {
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem; 
  private boolean earlyExit; 
  
  /**
   * Sets algae intake rollers and pivot angle.
   * @param algaeIntakeSubsystem
   */
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

  /**
   * If not interrupted, intake doesn't move back to default and keeps sucking. Otherwise, intake will stop sucking and return back to default angle.
   */
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      algaeIntakeSubsystem.setAngle(Rotation2d.fromDegrees(60));
      algaeIntakeSubsystem.setSpeed(0.9);
    }
    else{
      algaeIntakeSubsystem.setAngle(Constants.AlgaeIntake.DEFAULT_ANGLE); 
      algaeIntakeSubsystem.setSpeed(0);
    }
  }

  /**
   * Checks if piece is detected in the intake. Runs check on init and throughout command. 
   */
  @Override
  public boolean isFinished() {
    return earlyExit || algaeIntakeSubsystem.isPieceSensorActive(); 
  }
}
