package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

public class Watchdog {

  /**
   * start and end refer to the bounds of the system needing the watchdog (ex. the
   * min and max angle of a Pivot)
   * the current refers to the current position of the system (ex. for pivot it
   * would be the current angle)
   */
  private double start, end;
  private DoubleSupplier current;

  public Watchdog(double start, double end, DoubleSupplier current) {
    this.start = start;
    this.end = end;
    this.current = current;
  }

  /**
   * 
   * @returns whether or not bounds were crossed (ex. will return true if
   *          everything is good)
   *          fixes wrapping issue with many pivoting systems (ex. if Pivot
   *          wrapped around from 360 -> 0, basic watchdog code would break)
   */
  public boolean checkWatchdog() {
    return checkWatchdog(current.getAsDouble());
  }

  /**
   * 
   * @returns whether or not bounds were crossed (ex. will return true if
   *          everything is good)
   *          fixes wrapping issue with many pivoting systems (ex. if Pivot
   *          wrapped around from 360 -> 0, basic watchdog code would break)
   */
  public boolean checkWatchdog(double in) {
    if (end > start) {
      return in < end && in > start;
    } else {
      return in < end || in > start;
    }
  }
}
