package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
 
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);

  private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  private final LEDPattern scrollingrainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing).atBrightness(Percent.of(50));


  public LEDSubsystem() {
    led = new AddressableLED(Constants.LED.LED_PORT);
    buffer = new AddressableLEDBuffer(Constants.LED.LED_LENGTH);

    led.setLength(Constants.LED.LED_LENGTH);
    

    Color orange = new Color(255, 20, 0);
    LEDPattern orangePattern = LEDPattern.solid(orange).atBrightness(Percent.of(50));
    orangePattern.applyTo(buffer);
    led.setData(buffer);
    led.start();
  }

  /**
   * Set the LED pattern
   * @param pattern
   */
  public void setPattern(LEDPattern pattern) {
    pattern.applyTo(buffer);
    led.setData(buffer);
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
    // Color pink = new Color(255, 0, 20);
    // Color orange = new Color(255, 20, 0);


    // LEDPattern red = LEDPattern.solid(orange).atBrightness(Percent.of(50));
    // scrollingGay.applyTo(buffer);
    // // red.applyTo(buffer);

    // // setPattern(LEDPattern.solid(Color.kRed));
    // led.setData(buffer);
  }
}