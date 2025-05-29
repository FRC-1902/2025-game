// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator.ElevatorConstants.Position;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class ElevaterSim implements ElevatorBase {

    Position targetPosition; 
    boolean locked; 
    ElevatorSim elevatorSim; 
    PIDController pid;
    Pose3d stageOne, stageTwo; 
    DCMotor gearbox; 

    public ElevaterSim(){

        gearbox = DCMotor.getNEO(2); 
          
        elevatorSim = new ElevatorSim(gearbox, ElevatorConstants.SimulationConstants.ElevatorSimSetup.GEARING,
                ElevatorConstants.SimulationConstants.ElevatorSimSetup.CARRIAGE_MASS,
                ElevatorConstants.SimulationConstants.ElevatorSimSetup.DRUM_RADIUS,
                ElevatorConstants.SimulationConstants.ElevatorSimSetup.MIN_HEIGHT,
                ElevatorConstants.SimulationConstants.ElevatorSimSetup.MAX_HEIGHT,
                ElevatorConstants.SimulationConstants.ElevatorSimSetup.SIMULATE_GRAVITY,
                ElevatorConstants.SimulationConstants.ElevatorSimSetup.STARTING_HEIGHT);

        pid = new PIDController(ElevatorConstants.PIDConstants.kP, ElevatorConstants.PIDConstants.kI, ElevatorConstants.PIDConstants.kD); 
        stageOne = new Pose3d(new Translation3d(), new Rotation3d()); 
        stageTwo = new Pose3d(new Translation3d(), new Rotation3d()); 

        targetPosition = ElevatorConstants.Position.HOME; 
        resetPID();
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
        double restingPos = ElevatorConstants.Position.HOME.getHeight(); 
        double currentPos = targetPosition.getHeight(); 
        stageOne = new Pose3d(new Translation3d(restingPos, restingPos, (restingPos + currentPos) * 0.5 ), new Rotation3d()); // its kinda just as shrimple as that 
        stageTwo = new Pose3d(new Translation3d(restingPos, restingPos, restingPos + currentPos), new Rotation3d()); 

        Logger.recordOutput("Elevator", stageOne); 
        Logger.recordOutput("EndEffector", stageTwo); 
        //Logger.recordOutput("EndEffector", new Pose3d[]{stageOne, stageTwo});
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

        Logger.recordOutput("TargetElevatorPosition", targetPosition); 
        
        elevatorSim.setInputVoltage(pidCalc());
        updateTelemetry();
    };

}
