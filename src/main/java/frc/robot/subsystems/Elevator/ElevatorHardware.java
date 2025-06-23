// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Elevator.ElevatorConstants.Position;
import frc.robot.subsystems.Watchdog;

/** Add your docs here. */
public class ElevatorHardware implements ElevatorBase {

    SparkMax leftMotor, rightMotor; 
    Servo servo; 
    DigitalInput limitSwitch; 
    PIDController pid; 
    Position targetPosition; 
    double unlockTime; 
    double climbLockTime;
    Alert badStart, boundsAlert, servoAlert; 
    Watchdog elevatorWatchdog;

    public ElevatorHardware(){
        leftMotor = new SparkMax(ElevatorConstants.MotorIDs.LEFT_MOTOR_ID, MotorType.kBrushless); 
        rightMotor = new SparkMax(ElevatorConstants.MotorIDs.RIGHT_MOTOR_ID, MotorType.kBrushless); 

        servo = new Servo(ElevatorConstants.MotorIDs.SERVO_ID); 

        limitSwitch = new DigitalInput(ElevatorConstants.MotorIDs.LIMIT_SWITCH_ID); 

        pid = new PIDController(ElevatorConstants.PIDConstants.kP,ElevatorConstants.PIDConstants.kI,ElevatorConstants.PIDConstants.kD); 

        configureMotors();

        targetPosition = ElevatorConstants.Position.MIN;
    
        servo.setAngle(ElevatorConstants.CONVERSIONS.UNLOCK_ANGLE);
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

        configOne.encoder.positionConversionFactor(ElevatorConstants.CONVERSIONS.CONVERSION_FACTOR);
        configTwo.encoder.positionConversionFactor(ElevatorConstants.CONVERSIONS.CONVERSION_FACTOR);

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
        return 0.001 > Math.abs(servo.getAngle() - ElevatorConstants.CONVERSIONS.LOCK_ANGLE);
    };

    @Override
    public void setLocked(boolean lock) {
        if (lock) {
            servo.setAngle(ElevatorConstants.CONVERSIONS.LOCK_ANGLE);
        } else {
            unlockTime = Timer.getFPGATimestamp();
            servo.setAngle(ElevatorConstants.CONVERSIONS.UNLOCK_ANGLE);
        }
    }

    @Override
    public void resetPID(){
        pid.reset();
    };

  /**
   * go to the bottom of the elevator to re-home
   */
  private double home() {
    if (!limitSwitchTriggered()) {
      return -0.2; // TODO: change speed
    } else {
      leftMotor.getEncoder().setPosition(0);
      rightMotor.getEncoder().setPosition(0);
      badStart.set(false);
      
      return 0;
    }
  }

    private double climb() {
        if (!limitSwitchTriggered()) {
          climbLockTime = Timer.getFPGATimestamp(); // XXX; fix me
          return -0.5; // Move down at half speed
        } else {
          // When limit switch is triggered, lock the elevator
          if (!isLocked()) {
            setLocked(true);
          }
          
          // Continue applying slight downward pressure for a short time after locking
          if (Timer.getFPGATimestamp() - climbLockTime < 0.2) {
            return -0.35; // Gentle downward pressure
          } else {
            return 0; // Stop motor after the settling time
          }
        }
      }

    private double calcPID() {
        if (!isLocked() && Timer.getFPGATimestamp() - unlockTime > 0.3) {
          return pid.calculate(getPosition()) + ElevatorConstants.PIDConstants.kF + ElevatorConstants.PIDConstants.kS * Math.signum(pid.getSetpoint() - getPosition());
        } else {
          return 0;
        }    
      }

    @Override
    public void update(ElevatorBaseInputs inputs){
        double power; 

        switch (targetPosition) {
            case HOLD:
              power = 0;
              break;
            case HOME:
              if (isLocked()) {
                setLocked(false);
              }
              power = home();
              break;
            case CLIMB_DOWN:
              power = climb();
              break;
            default:
              if (isLocked()) {
                setLocked(false);
              }
              power = calcPID();
              break;
          }

        leftMotor.set(power);
        rightMotor.set(power); 

        inputs.atSetpoint = pid.atSetpoint(); 
        inputs.currentPosition = getPosition(); 
        inputs.limitSwitchTriggered = limitSwitchTriggered(); 
        inputs.isLocked = isLocked(); 
    };
}