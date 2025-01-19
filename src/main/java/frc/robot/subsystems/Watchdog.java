// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class Watchdog {

    private double start, end;
    private DoubleSupplier current;

    public Watchdog(double start, double end, DoubleSupplier current) {
        this.start = start;
        this.end = end;
        this.current = current;
    }

    /**
     * 
     * @returns whether or not bounds were crossed (ex. if Pivot is out of set min
     *          or max)
     *          fixes wrapping issue with many pivoting systems (ex. if Pivot
     *          wrapped around from 360 -> 0, basic watchdog code would break)
     */
    public boolean checkWatchingdog() {
        if (end > start) {
            return current.getAsDouble() < end && current.getAsDouble() > start;
        } else {
            return current.getAsDouble() > end || current.getAsDouble() < start;
        }
    }
}
