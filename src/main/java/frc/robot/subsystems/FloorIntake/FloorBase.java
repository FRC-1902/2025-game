// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FloorIntake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.epilogue.Logged;

/** Add your docs here. */
@Logged
public interface FloorBase {

    @Logged
    public class FloorBaseInputs{
        boolean hasCoral; 
        boolean atSetpoint; 
        Rotation2d currentAngle; 
        Rotation2d targetAngle; 
    }

    public void setSpeed(double speed);

    public Rotation2d getAngle();

    public void setAngle(Rotation2d angle);

    public boolean hasCoral();

    public void resetPID();

    public void update(FloorBaseInputs inputs);
}
