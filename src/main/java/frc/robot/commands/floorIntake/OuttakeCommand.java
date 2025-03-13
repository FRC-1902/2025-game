package frc.robot.commands.floorIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntakeSubsystem;

public class OuttakeCommand extends Command {
  private final FloorIntakeSubsystem floorIntakeSubsystem;

  /**
   * runs floor intake rollers outward when command is scheduled
   * @param floorIntakeSubsystem
   */
  public OuttakeCommand(FloorIntakeSubsystem floorIntakeSubsystem) {
    this.floorIntakeSubsystem = floorIntakeSubsystem;

    addRequirements(floorIntakeSubsystem);
  }

  @Override
  public void initialize() {
    floorIntakeSubsystem.setSpeed(-0.7); 
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    floorIntakeSubsystem.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
