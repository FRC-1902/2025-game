// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

/** Add your docs here. */
public interface EndBase {

    public class EndBaseInputs {
        boolean hasCoralFront;
        boolean hasCoralBack;
    }

    public void setSpeed(double speed);

    public boolean hasCoralFront();

    public boolean hasCoralBack();

    public void update(EndBaseInputs inputs);
}
