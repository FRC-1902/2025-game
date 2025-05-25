// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.Position;

/** Add your docs here. */
public class ElevatorHardware implements ElevatorBase {

    SparkMax leftMotor, rightMotor; 
    Servo servo; 
    DigitalInput limitSwitch; 
    PIDController pid; 
    Position targetPosition; 
    double unlockTime; 

    public ElevatorHardware(){
        leftMotor = new SparkMax(0, null); 
        rightMotor = new SparkMax(0, null); 

        servo = new Servo(0); 

        limitSwitch = new DigitalInput(0); 

        pid = new PIDController(0,0,0); 

        configureMotors();
    }

    private void configureMotors() {
        SparkBaseConfig configOne = new SparkMaxConfig();
        SparkBaseConfig configTwo = new SparkMaxConfig();

        configOne.idleMode(IdleMode.kCoast);
        configOne.smartCurrentLimit(50);
        configOne.inverted(true); // todo: switch inverted
        configOne.voltageCompensation(12);

        configTwo.idleMode(IdleMode.kCoast);
        configTwo.smartCurrentLimit(50);
        configTwo.inverted(false); // todo: switch inverted
        configTwo.voltageCompensation(12);

        configOne.encoder.positionConversionFactor(Constants.Elevator.CONVERSION_FACTOR);
        configTwo.encoder.positionConversionFactor(Constants.Elevator.CONVERSION_FACTOR);

        // ResetSafeParameters not well documented
        leftMotor.configure(configOne, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.configure(configTwo, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public double getPosition(){
        return (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getPosition()) * 0.5; 
    };

    @Override
    public void setPosition(Position targetPosition){
        this.targetPosition = targetPosition; 
        pid.setSetpoint(targetPosition.getHeight());
    };

    @Override
    public boolean limitSwitchTriggered(){
        return limitSwitch.get(); 
    }

    @Override
    public boolean atSetpoint(){
        return pid.atSetpoint();
    };

    @Override
    public boolean isLocked(){
        return 0.001 > Math.abs(servo.getAngle() - Constants.Elevator.LOCK_ANGLE);
    };

    @Override
    public void setLocked(boolean lock) {
        if (lock) {
            servo.setAngle(Constants.Elevator.LOCK_ANGLE);
        } else {
            unlockTime = Timer.getFPGATimestamp();
            servo.setAngle(Constants.Elevator.UNLOCK_ANGLE);
        }
    }

    @Override
    public void resetPID(){
        pid.reset();
    };

    public void elePowerCalc(){
        double power = pid.calculate(getPosition()) + Constants.Elevator.kF + Constants.Elevator.kS * Math.signum(pid.getSetpoint() - getPosition());

        leftMotor.set(power);
        rightMotor.set(power); 
    }

    @Override
    public void update(ElevatorBaseInputs inputs){

        inputs.atSetpoint = pid.atSetpoint(); 
        inputs.currentPosition = targetPosition; 
        inputs.unlockTime = unlockTime; 
        inputs.limitSwitchTriggered = limitSwitchTriggered(); 
        inputs.isLocked = isLocked(); 
    };
}
