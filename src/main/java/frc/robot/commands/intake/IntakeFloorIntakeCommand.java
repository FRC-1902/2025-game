package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import static edu.wpi.first.units.Units.Percent;

public class IntakeFloorIntakeCommand extends Command {
  private final FloorIntakeSubsystem floorIntakeSubsystem; 
  private final LEDSubsystem led;
  private double endTime = 0;
  private final LEDPattern color = LEDPattern.solid(new Color(0, 255, 0)).atBrightness(Percent.of(50)); // Green color

  /** Creates a new IntakeFloorIntakeCommand. */
  public IntakeFloorIntakeCommand(FloorIntakeSubsystem floorIntakeSubsystem, LEDSubsystem led) {
    this.floorIntakeSubsystem = floorIntakeSubsystem; 
    this.led = led;

    led.registerPattern(() -> (Timer.getFPGATimestamp() - endTime) < 0.25, color);

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
