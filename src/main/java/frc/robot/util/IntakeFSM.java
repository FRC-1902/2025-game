// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.Constants.Elevator.Position;
import frc.robot.Constants.FloorIntake;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class IntakeFSM {

    public enum States{
        IN, 
        OUT, 
        TRANS_IN, 
        TRANS_OUT;
    }

    private States currentState;
    private Position targetPosition;
    private FloorIntakeSubsystem floorIntakeSubsystem; 
    private ElevatorSubsystem elevatorSubsystem;
    private Rotation2d targetAngle; 

    

    public IntakeFSM(FloorIntakeSubsystem floorIntakeSubsystem, ElevatorSubsystem elevatorSubsystem){
        currentState = States.IN;
        this.floorIntakeSubsystem = floorIntakeSubsystem; 
        this.elevatorSubsystem = elevatorSubsystem;
    }

    public void eventHandler(States desiredState){
        desiredState = currentState;
        
        switch(getCurrentState()){

            case IN: 
            getInState();
            while(!elevatorSubsystem.isAtPosition(targetPosition)){
                elevatorSubsystem.setPosition(targetPosition);
            }
            floorIntakeSubsystem.setAngle(targetAngle);

            case OUT: 
            getOutState();
            floorIntakeSubsystem.setAngle(targetAngle);

            case TRANS_IN: 
            getTransInState();
            while(!elevatorSubsystem.isAtPosition(targetPosition)){
                elevatorSubsystem.setPosition(Position.MIN);
            }
            floorIntakeSubsystem.setAngle(targetAngle);
            
            case TRANS_OUT:
            getTransOutState();
            elevatorSubsystem.setPosition(targetPosition);
            floorIntakeSubsystem.setAngle(targetAngle);
        }
    }

    public States getCurrentState(){
        return currentState;
    }

    //GOAL POSITION
    private void getInState(){
        targetAngle = FloorIntake.DEFAULT_ANGLE;
        targetPosition = Position.MIN;
    }

    //GOAL POSITION
    private void getOutState(){
       targetAngle = FloorIntake.FLOOR_ANGLE;
    }

    //HOW WE GET THEIR
    private void getTransInState(){
        if(targetPosition != Position.L1 || targetPosition != Position.L2 || targetPosition != Position.L3){
            targetAngle = FloorIntake.DEFAULT_ANGLE;
        }
        else{
            targetAngle = FloorIntake.ELEVATOR_ANGLE;
        }
    }

    //SPECIAL OUT CASE
    private void getTransOutState(){
        targetAngle = FloorIntake.FLOOR_ANGLE; 
        targetPosition = Position.MIN;
    }
}
   


