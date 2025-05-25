// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator.ElevatorBase.ElevatorBaseInputs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Elevator.Position;

public class ElevatorSubsystem extends SubsystemBase {

  ElevatorBase elevatorBase; 
  RoutineManager routineManager; 
  ElevatorBaseInputs inputs; 
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    inputs = new ElevatorBaseInputs(); 
    if(Robot.isReal()){
      elevatorBase = new ElevatorHardware();
    }
    else{
      elevatorBase = new ElevaterSim(); 
    }
  }

  public Command setPosition(Position targetPosition){
    return new InstantCommand(() -> inputs.targetPosition = targetPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorBase.update(inputs);
    routineManager.gangCanYouManageMyParkingLot(inputs.targetPosition);
  }
}
