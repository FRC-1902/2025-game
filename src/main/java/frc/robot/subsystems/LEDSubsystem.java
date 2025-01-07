package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
 
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public LEDSubsystem() {
    led = new AddressableLED(Constants.LED.LED_PORT);
    buffer = new AddressableLEDBuffer(Constants.LED.LED_LENGTH);

    led.setLength(Constants.LED.LED_LENGTH);
    led.setData(buffer);
    led.start();
  }

  /**
   * Set the LED pattern
   * @param pattern
   */
  public void setPattern(LEDPattern pattern) {
    pattern.applyTo(buffer);
  }

  /**
   * Set the LED pattern with a command, Usage: ledSubsystem.setPatternCommand(LEDPattern.solid(Color.kRed))
   * @param pattern
   * @return
   */
  public Command getLedCommand (LEDPattern pattern) {
    return Commands.runOnce(() -> setPattern(pattern), this);
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED pattern, needs to run periodically for patterns with animations
    led.setData(buffer);
  }
}