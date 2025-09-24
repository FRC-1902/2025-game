// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FloorIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FloorIntake.FloorBase.FloorBaseInputs;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.CoralDetectorSim;
import frc.robot.Robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;
@Logged
public class FloorSubsystem extends SubsystemBase {

  FloorBase floorBase;
  FloorBaseInputs inputs;
  BooleanSupplier safe; 
  SwerveSubsystem swerveSubsystem; 
  CoralDetectorSim coralDetectorSim; 

  /** Creates a new FloorSubsystem. */
  public FloorSubsystem(BooleanSupplier safe, SwerveSubsystem swerveSubsystem, CoralDetectorSim coralDetectorSim) {
    inputs = new FloorBaseInputs();
    if (Robot.isReal()) {
      floorBase = new FloorHardware();
    } else {
      floorBase = new FloorSim(swerveSubsystem, coralDetectorSim);
    }

    this.safe = safe; 
  }

  public Command runRollers(double speed) {
    return Commands.runEnd(() -> floorBase.setSpeed(speed), () -> floorBase.setSpeed(0));
  }

  public Command setPivotAngle(Rotation2d angle) {
    return Commands.either(
        run(() -> floorBase.setAngle(Rotation2d.fromDegrees(FloorConstants.Positions.ELEVATOR_ANGLE)))
            .until(safe)
            .andThen(() -> floorBase.setAngle(angle)),
        run(() -> floorBase.setAngle(angle)),
        () -> (Math.abs(angle.getDegrees() - FloorConstants.Positions.DEFAULT_ANGLE.getDegrees()) < 0.01));
  }

  public void resetPID(){
    floorBase.resetPID();
  }

  public Rotation2d currentAngle(){
    return inputs.currentAngle; 
  }

  public boolean atSetpoint(){
    return inputs.atSetpoint; 
  }

  public boolean hasCoral(){
    return inputs.hasCoral; 
  }

  public void setCoral(boolean coral){
    inputs.hasCoral = coral; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    floorBase.update(inputs);
    Logger.recordOutput("FloorIntake/TargetAngle", inputs.targetAngle.getDegrees());
    Logger.recordOutput("FloorIntake/CurrentAngle", inputs.currentAngle.getDegrees());
  }
}
