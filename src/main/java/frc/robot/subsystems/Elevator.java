// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;


public class Elevator extends SubsystemBase {
  private SparkMax leftMotor, rightMotor; 
  private SparkBaseConfig configOne, configTwo; 
  private DigitalInput leftBottomSwitch, rightBottomSwitch, leftTopSwitch, rightTopSwitch; 
  private LoggedNetworkBoolean LBLogged, RBLogged, LTLogged, RTLogged; 
  private LoggedDashboardChooser<Boolean> climberDisableChooser; 
  private Direction targetDirection; 

  
  public enum Direction{
    UP, DOWN, STOP
  }
  /** Creates a new Elevator. */
  public Elevator() {
    leftMotor = new SparkMax(Constants.Elevator.LEFT_MOTOR_ID, MotorType.kBrushless); 
    rightMotor = new SparkMax(Constants.Elevator.RIGHT_MOTOR_ID, MotorType.kBrushless);

    configOne = new SparkMaxConfig(); 
    configTwo = new SparkMaxConfig(); 

    configureMotors(); 
    
    leftBottomSwitch = new DigitalInput(Constants.Elevator.LB_SWITCH_PORT);
    rightBottomSwitch = new DigitalInput(Constants.Elevator.RB_SWITCH_PORT);
    leftTopSwitch = new DigitalInput(Constants.Elevator.LT_SWITCH_PORT);
    rightTopSwitch = new DigitalInput(Constants.Elevator.RT_SWITCH_PORT);

    targetDirection = Direction.STOP; 

    configureNTData();

  }

  private void configureMotors(){
    configOne.idleMode(IdleMode.kBrake);
    configOne.smartCurrentLimit(30);
    configOne.inverted(true);
    configOne.voltageCompensation(12);

    configTwo.idleMode(IdleMode.kBrake);
    configTwo.smartCurrentLimit(30);
    configTwo.inverted(false);
    configTwo.voltageCompensation(12); 
    configTwo.follow(Constants.Elevator.LEFT_MOTOR_ID); 

  }

  public void setDirection(Direction targetDirection){
    this.targetDirection = targetDirection; 
  }

  public boolean atSetpoint() {
    switch (targetDirection) {
      case UP:
        return !leftTopSwitch.get() && !rightTopSwitch.get();
      case DOWN:
        return !leftBottomSwitch.get() && !rightBottomSwitch.get();
      default:
        return false;
    }
  }

   private void configureNTData() {
    LBLogged = new LoggedNetworkBoolean("Climber/Left Bottom Switch");
    LTLogged = new LoggedNetworkBoolean("Climber/Left Top Switch");
    RBLogged = new LoggedNetworkBoolean("Climber/Right Bottom Switch");
    RTLogged = new LoggedNetworkBoolean("Climber/Right Top Switch");

    climberDisableChooser = new LoggedDashboardChooser<>("Climber/Climber Disable");
    climberDisableChooser.addDefaultOption("Normal Climbing", false);
    climberDisableChooser.addOption("STOP CLIMBER", true);
  }

  private void putNTData() {
    LBLogged.set(!leftBottomSwitch.get());
    LTLogged.set(!leftTopSwitch.get());
    RBLogged.set(!rightBottomSwitch.get());
    RTLogged.set(!rightTopSwitch.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    putNTData();

    if (Boolean.TRUE.equals(climberDisableChooser.get())) {
      leftMotor.set(0.0);
      rightMotor.set(0.0);
      return;
    }
    
    switch (targetDirection) {
      case UP:
        if (leftTopSwitch.get()) {
          leftMotor.set(0.75);
        } else {
          leftMotor.set(0.0);
        }
        if (rightTopSwitch.get()) {
          rightMotor.set(0.75);
        } else {
          rightMotor.set(0.0);
        }
        break;
      case DOWN:
        if (leftBottomSwitch.get()) {
          leftMotor.set(-0.75);
        } else {
          leftMotor.set(0.0);
        }
        if (rightBottomSwitch.get()) {
          rightMotor.set(-0.75);
        } else {
          rightMotor.set(0.0);
        }
        break;
      case STOP:
        leftMotor.set(0);
        rightMotor.set(0);
        break;
      default:
        break;
    }
  }
}
