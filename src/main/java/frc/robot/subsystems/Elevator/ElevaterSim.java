// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import frc.robot.Robot;
import frc.robot.Constants.Elevator.Position;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ElevaterSim implements ElevatorBase {

ElevatorBaseInputs inputs; 
    Position targetPosition; 
    ElevatorSim elevatorSim; 

    public ElevaterSim(){
        inputs = new ElevatorBaseInputs(); 
        // Sim setup            
    }

    public double getPosition(){
       return inputs.currentPosition.getHeight();
    };

    public void setPosition(Position position){
        targetPosition = position; 
    };

    public boolean limitSwitchTriggered(){
        return inputs.limitSwitchTriggered; 
    }; 

    public boolean atSetpoint(){
        return inputs.atSetpoint; 
    };

    public void resetPID(){};

    public boolean isLocked(){
        return inputs.isLocked; 
    }; 

    public void setLocked(boolean lock){
        inputs.isLocked = lock; 
    }; 

    public void update(ElevatorBaseInputs inputs){
        if(Robot.isReal()) return; 
        //Sim Logic here 
    };

}
