package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class IndexFloorIntakeCommand extends Command {
  private final FloorIntakeSubsystem floorIntakeSubsystem;
  private final EndEffectorSubsystem endEffectorSubsystem;
  
  /** Creates a new IndexFloorIntakeCommand. */
  public IndexFloorIntakeCommand(FloorIntakeSubsystem floorIntakeSubsystem, EndEffectorSubsystem endEffectorSubsystem){
    this.floorIntakeSubsystem = floorIntakeSubsystem; 
    this.endEffectorSubsystem = endEffectorSubsystem;

    addRequirements(floorIntakeSubsystem, endEffectorSubsystem);
  }

  @Override
  public void initialize() {
    //floorIntakeSubsystem.setSpeed(0.2); // todo: find indexing speed
    //endEffectorSubsystem.setSpeed(0.2); // todo: find indexing speed
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    //floorIntakeSubsystem.setSpeed(0);
    //endEffectorSubsystem.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    // XXX: may not want front piece sensor true check here, but it probably should be
    return !floorIntakeSubsystem.pieceSensorActive() && !endEffectorSubsystem.isBackPieceSensorActive() && endEffectorSubsystem.isFrontPieceSensorActive();
  }
}
