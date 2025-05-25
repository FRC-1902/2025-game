// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Elevator.Position;

/** Add your docs here. */
public class RoutineManager extends ElevatorHardware {

    ElevatorBaseInputs inputs;
    Position targetPosition;

    public RoutineManager(Position targetPosition) {
        this.targetPosition = targetPosition;
        inputs = new ElevatorBaseInputs();
    }

    public void gangCanYouManageMyParkingLot(Position targetPosition) {
        switch (targetPosition) {

            case CLIMB_DOWN:
                if (!inputs.limitSwitchTriggered) {
                    setPowers(-0.5);
                } else {
                    setLocked(true);
                    inputs.isLocked = true;
                    protectTheGoodGrass();
                    break;
                }
            case CLIMB_UP:
                setLocked(false);
                inputs.isLocked = false;
                setPowers(0.5);

            default:
                elePowerCalc();
        }
    }

    private void protectTheGoodGrass() {
        // Continue applying slight downward pressure for a short time after locking
        if (Timer.getFPGATimestamp() - inputs.climbLockTime < 0.2) {
            setPowers(-0.35);
             // Gentle downward pressure
        } else {
            setPowers(0);
             // Stop motor after the settling time
        }
    }

    public void setPowers(double powers) {
        leftMotor.set(powers);
        rightMotor.set(powers);
    }
}
