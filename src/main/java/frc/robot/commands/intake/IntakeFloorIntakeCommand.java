package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntakeSubsystem;

public class IntakeFloorIntakeCommand extends Command {
  private final FloorIntakeSubsystem floorIntakeSubsystem; 

  /** Creates a new IntakeFloorIntakeCommand. */
  public IntakeFloorIntakeCommand(FloorIntakeSubsystem floorIntakeSubsystem) {
    this.floorIntakeSubsystem = floorIntakeSubsystem; 

    addRequirements(floorIntakeSubsystem);
  }

  @Override
  public void initialize() {
    //floorIntakeSubsystem.setSpeed(1);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    //floorIntakeSubsystem.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return floorIntakeSubsystem.pieceSensorActive();
  }
}
