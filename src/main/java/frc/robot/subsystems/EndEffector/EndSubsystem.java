// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EndEffector.EndBase.EndBaseInputs;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;

public class EndSubsystem extends SubsystemBase {

  EndBase endBase;
  EndBaseInputs inputs;

  /** Creates a new EndSubsystem. */
  public EndSubsystem() {
    inputs = new EndBaseInputs();
    if (Robot.isReal()) {
      endBase = new EndHardware();
    } else {
      endBase = new EndSim();
    }
  }

  public Command runRollers(double speed) {
    return runEnd(() -> endBase.setSpeed(speed), () -> endBase.setSpeed(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    endBase.update(inputs);
  }
}
