// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.AlgaeIntake.AlgaeBase.AlgaeBaseInputs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlgaeSubsystem extends SubsystemBase {

  public AlgaeBase algaeBase; 
  public AlgaeBaseInputs inputs;

  // Here bc why not bro 
  // public final Trigger hasAlgae = new Trigger(() -> inputs.hasAlgae);
  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {
    inputs = new AlgaeBaseInputs(); 
    if(Robot.isReal()){
    algaeBase = new AlgaeHardware();}
    else{
      algaeBase = new AlgaeSim();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    algaeBase.update(inputs);
  }

  public Command runRollers(double speed) {
    return runEnd(() -> algaeBase.setSpeed(speed), () -> algaeBase.setSpeed(0));
  }

  public Command setPivotAngle(Rotation2d angle) {
    return run(() -> algaeBase.setAngle(angle));
  }
}
