package frc.robot.commands.floorIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import static edu.wpi.first.units.Units.Percent;

public class IntakeCommand extends Command {
  private final FloorIntakeSubsystem floorIntakeSubsystem; 
  private final LEDSubsystem led;
  private double endTime = 0;
  private final LEDPattern color = LEDPattern.solid(new Color(0, 255, 0)).atBrightness(Percent.of(50)); // Green color

  /**
   * runs floor intake rollers inward until piece is detected.
   * <p>the rollers will stay on for a successful intake for other compositions to handle it</p>
   * @param floorIntakeSubsystem
   * @param led
   */
  public IntakeCommand(FloorIntakeSubsystem floorIntakeSubsystem, LEDSubsystem led) {
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
    if (interrupted)
      floorIntakeSubsystem.setSpeed(0);
    if (!interrupted) {
      ControllerSubsystem.getInstance().vibrate(ControllerName.DRIVE, 300, 1);
      ControllerSubsystem.getInstance().vibrate(ControllerName.MANIP, 300, 1);
    }
    endTime = Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    return floorIntakeSubsystem.pieceSensorActiveFiltered();
  }
}
