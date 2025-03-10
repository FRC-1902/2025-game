package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntakeSubsystem;

public class OuttakeFloorIntakeCommand extends Command {
  private final FloorIntakeSubsystem floorIntakeSubsystem;

  /** Creates a new OuttakeFloorIntakeCommand. */
  public OuttakeFloorIntakeCommand(FloorIntakeSubsystem floorIntakeSubsystem) {
    this.floorIntakeSubsystem = floorIntakeSubsystem;

    addRequirements(floorIntakeSubsystem);
  }

  @Override
  public void initialize() {
    floorIntakeSubsystem.setSpeed(-0.7); // todo: find outtake speed
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
