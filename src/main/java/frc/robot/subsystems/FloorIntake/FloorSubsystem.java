// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FloorIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FloorIntake.FloorBase.FloorBaseInputs;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

@Logged
public class FloorSubsystem extends SubsystemBase {

  FloorBase floorBase;
  FloorBaseInputs inputs;

  /** Creates a new FloorSubsystem. */
  public FloorSubsystem() {
    inputs = new FloorBaseInputs();
    if (Robot.isReal()) {
      floorBase = new FloorHardware();
    } else {
      floorBase = new FloorSim();
    }
  }

  public Command runRollers(double speed) {
    return runEnd(() -> floorBase.setSpeed(speed), () -> floorBase.setSpeed(0));
  }

  public Command setPivotAngle(Rotation2d angle) {
    return new InstantCommand(() -> floorBase.setAngle(angle));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    floorBase.update(inputs);
    Logger.recordOutput("FloorIntake/TargetAngle", inputs.targetAngle.getDegrees());
    Logger.recordOutput("FloorIntake/CurrentAngle", inputs.currentAngle.getDegrees());

  }
}
