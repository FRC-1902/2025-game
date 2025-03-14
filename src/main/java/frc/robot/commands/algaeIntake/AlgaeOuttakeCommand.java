package frc.robot.commands.algaeIntake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeOuttakeCommand extends Command {
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem; 
  
  /**
   * When called spits out until piece is no longer detected.
   * @param algaeIntakeSubsystem
   */
  public AlgaeOuttakeCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;  
    addRequirements(algaeIntakeSubsystem);
  }

  @Override
  public void initialize() {
    algaeIntakeSubsystem.setSpeed(-0.6);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    algaeIntakeSubsystem.setAngle(Constants.AlgaeIntake.DEFAULT_ANGLE); 
    algaeIntakeSubsystem.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return !algaeIntakeSubsystem.isPieceSensorActive(); 
  }
}
