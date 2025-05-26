// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import frc.robot.Robot;
import frc.robot.Constants.Elevator.Position;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class ElevaterSim implements ElevatorBase {

    Position targetPosition; 
    boolean locked; 
    ElevatorSim elevatorSim; 
    PIDController pid;
    Pose3d stageOne, stageTwo; 

    public ElevaterSim(){
        // Sim setup        
        elevatorSim = new ElevatorSim(null, getPosition(), getPosition(), getPosition(), getPosition(), getPosition(), locked, getPosition(), null); 
        
        pid = new PIDController(getPosition(), getPosition(), getPosition()); 
        stageOne = new Pose3d(null, null); 
        stageTwo = new Pose3d(null, null); 

        resetPID();
        //TODO: values
    }

    public double getPosition(){
       return elevatorSim.getPositionMeters();
    };

    public void setPosition(Position position){
        targetPosition = position; 
    };

    public boolean limitSwitchTriggered(){
        return elevatorSim.wouldHitLowerLimit(getPosition()); // TODO: Some number for the lower limit of the elevator
    }; 

    public boolean atSetpoint(){
        return 0.01 >= Math.abs(getPosition() - targetPosition.getHeight()); 
    };

    public void resetPID(){
        pid.reset();
    };

    public boolean isLocked(){
        return locked; 
    }; 

    public void setLocked(boolean lock){
        locked = lock; 
    }; 

    private void updateTelemetry(){
        double restingPos = 0; 
        double currentPos = elevatorSim.getPositionMeters(); 
        stageOne = new Pose3d(new Translation3d(restingPos, restingPos, restingPos + currentPos), new Rotation3d()); 
        stageTwo = new Pose3d(new Translation3d(restingPos, restingPos, restingPos + (2 * currentPos)), new Rotation3d()); 

        Logger.recordOutput("stageOne", stageOne); 
        Logger.recordOutput("stageTwo", stageTwo); 
    }

    private double pidCalc(){
        double kGConstant = 0;
        return pid.calculate(elevatorSim.getPositionMeters(), targetPosition.getHeight()) + kGConstant;
    }

    public void update(ElevatorBaseInputs inputs){
        if(Robot.isReal()) return; 
        //Sim Logic here 
        inputs.atSetpoint = atSetpoint(); 
        inputs.limitSwitchTriggered = limitSwitchTriggered(); 
        inputs.currentPosition = getPosition();
        inputs.targetPosition = targetPosition; 
        inputs.isLocked = locked; 
        
        elevatorSim.setInputVoltage(pidCalc());
        updateTelemetry();
    };

}
