package frc.robot.commands.floorIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntakeSubsystem;

public class OuttakeCommand extends Command {
  private final FloorIntakeSubsystem floorIntakeSubsystem;

  /**
   * Runs floor intake rollers outward when command is scheduled
   * Doesn't have end conditions, will continue to run
   * @param floorIntakeSubsystem
   */
  public OuttakeCommand(FloorIntakeSubsystem floorIntakeSubsystem) {
    this.floorIntakeSubsystem = floorIntakeSubsystem;

    addRequirements();
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
