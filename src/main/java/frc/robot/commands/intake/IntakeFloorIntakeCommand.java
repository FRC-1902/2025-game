package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import static edu.wpi.first.units.Units.Percent;

public class IntakeFloorIntakeCommand extends Command {
  private final FloorIntakeSubsystem floorIntakeSubsystem; 

  private double endTime = 0;

  /** Creates a new IntakeFloorIntakeCommand. */
  public IntakeFloorIntakeCommand(FloorIntakeSubsystem floorIntakeSubsystem) {
    this.floorIntakeSubsystem = floorIntakeSubsystem; 


    addRequirements(floorIntakeSubsystem);
  }

  @Override
  public void initialize() {
    
    floorIntakeSubsystem.setSpeed(1);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  floorIntakeSubsystem.setSpeed(0);
  endTime = Timer.getFPGATimestamp();
}

  @Override
  public boolean isFinished() {
    return floorIntakeSubsystem.pieceSensorActive();
  }
}
