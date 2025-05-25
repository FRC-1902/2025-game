// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import frc.robot.Constants.Elevator.Position;

/** Add your docs here. */
public interface ElevatorBase {

    public class ElevatorBaseInputs {
        boolean atSetpoint;
        boolean limitSwitchTriggered; 
        boolean isLocked; 
        Position currentPosition;
        Position targetPosition; 
        double unlockTime; 
        double climbLockTime; 
    }

    public double getPosition();

    public void setPosition(Position position);

    public boolean limitSwitchTriggered(); 

    public boolean atSetpoint();

    public void resetPID();

    public boolean isLocked(); 

    public void setLocked(boolean lock); 

    public void update(ElevatorBaseInputs inputs);
}
