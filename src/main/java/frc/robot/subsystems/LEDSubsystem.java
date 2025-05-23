package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// TODO: setup lengths and spacings
public class LEDSubsystem extends SubsystemBase {
 
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
  private final LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing).atBrightness(Percent.of(50));

  private List<KeyValue<BooleanSupplier, LEDPattern>> ledRegistry;

  private LEDPattern orangePattern = LEDPattern.solid(new Color(255, 20, 0)).atBrightness(Percent.of(50));

  public LEDSubsystem() {
    led = new AddressableLED(Constants.LED.LED_PORT);
    buffer = new AddressableLEDBuffer(Constants.LED.LED_LENGTH);

    led.setLength(Constants.LED.LED_LENGTH);
    led.start();

    ledRegistry = new ArrayList<>();
  }

  /**
   * Register a pattern based on some boolean supplier
   * The first registered boolean to become true sets as the pattern on the subsystem
   * @param condition
   * @param pattern
   */
  public void registerPattern(BooleanSupplier condition, LEDPattern pattern) {
    ledRegistry.add(new KeyValue<>(condition, pattern));
  }

  /**
   * Set the LED pattern
   * @param pattern
   */
  private void setPattern(LEDPattern pattern) {
    pattern.applyTo(buffer);
    led.setData(buffer);
  }

  @Override
  public void periodic() {
    // default color of orange
    LEDPattern selected = orangePattern;

    for (int i = 0; i < ledRegistry.size(); i++) {
      if (ledRegistry.get(i).getKey().getAsBoolean()) {
        selected = ledRegistry.get(i).getValue();
        break;
      }
    }
    setPattern(selected);
  }
}

class KeyValue<K, V> {
  private K key;
  private V value;

  public KeyValue(K key, V value) {
    this.key = key;
    this.value = value;
  }

  public K getKey() {
    return key;
  }

  public V getValue() {
    return value;
  }
}