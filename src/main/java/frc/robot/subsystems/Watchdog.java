// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class Watchdog extends SubsystemBase {
  private double start, end;
  private DoubleSupplier current;

  /** Creates a new Watchdog. */
  public Watchdog(double start, double end, DoubleSupplier current) {
    this.start = start;
    this.end = end;
    this.current = current;
  }

  /**
   * 
   * @returns whether bounds specified were violated or not
   */
  public boolean checkWatchDog() {
    if (end > start) {
      return current.getAsDouble() < end && current.getAsDouble() > start;
    } else {
      return current.getAsDouble() > end || current.getAsDouble() < start;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
