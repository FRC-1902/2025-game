// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffectorSubsystem extends SubsystemBase {

  private SparkMax rollerMotor; 
  private DigitalInput frontSensor, backSensor;
  private double targetSpeed; 

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem() {
    rollerMotor = new SparkMax(Constants.EndEffector.ROLLER_MOTOR_ID, MotorType.kBrushless);

    frontSensor = new DigitalInput(Constants.EndEffector.FRONT_SENSOR_CHANNEL);
    backSensor = new DigitalInput(Constants.EndEffector.BACK_SENSOR_CHANNEL);

    configureMotors();
  }

  private void configureMotors(){
    SparkBaseConfig rollerConfig = new SparkMaxConfig();
    // Roller Configs
    rollerConfig.idleMode(IdleMode.kCoast);
    rollerConfig.inverted(false);
    rollerConfig.disableFollowerMode(); 
    rollerConfig.secondaryCurrentLimit(30);
    rollerConfig.smartCurrentLimit(30);
    rollerConfig.voltageCompensation(12.00);
    // ResetSafeParameters subject to change; not well documented 
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  
  public boolean isFrontPieceSensorActive(){
    return frontSensor.get(); 
  }

  public boolean isBackPieceSensorActive(){
    return backSensor.get(); 
  }

  public void setSpeed(double targetSpeed){
    this.targetSpeed = targetSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!isFrontPieceSensorActive() && isBackPieceSensorActive()){
      setSpeed(targetSpeed);
    }
    else{
      setSpeed(0); 
    }
  }
}
