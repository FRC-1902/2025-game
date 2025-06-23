// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator.ElevatorBase.ElevatorBaseInputs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator.ElevatorConstants.Position;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.Watchdog;

public class ElevatorSubsystem extends SubsystemBase {

  ElevatorBase elevatorBase; 
  ElevatorBaseInputs inputs; 
  Alert badStart, boundsAlert, servoAlert; 
  Watchdog elevatorWatchdog; 
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    inputs = new ElevatorBaseInputs();
    if (Robot.isReal()) {
      elevatorBase = new ElevatorHardware();
    } else {
      elevatorBase = new ElevaterSim();
    }

    badStart = new Alert("Elevator start position wrong, limit switch not triggered", AlertType.kError);
    boundsAlert = new Alert("Elevator/Elevator out of bounds", AlertType.kError);
    servoAlert = new Alert("Elevator/Cannot Exit Climb, Servo is locked", AlertType.kWarning);

    if (!inputs.limitSwitchTriggered) {
      badStart.set(true);
    }

    elevatorWatchdog = new Watchdog(ElevatorConstants.Position.MAX.getHeight() + 0.01, ElevatorConstants.Position.MIN.getHeight() - 0.05, (() -> inputs.currentPosition));
  }

  public Command setPosition(Position targetPosition){
    return new InstantCommand(() -> elevatorBase.setPosition(targetPosition));
  }

  public void resetElevatorPID(){
    elevatorBase.resetPID();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorBase.update(inputs);

    if(inputs.targetPosition == Position.HOME){
        badStart.set(false);
    }
    else{
        badStart.set(true);
    }

    if(elevatorBase.isLocked() && inputs.targetPosition != ElevatorConstants.Position.CLIMB_DOWN){
        servoAlert.set(true);
    }
    else{
        servoAlert.set(false);
    }

    if(!elevatorWatchdog.checkWatchdog()){
        boundsAlert.set(true);
    }
    else{
        boundsAlert.set(true); 
    }
  }
}
