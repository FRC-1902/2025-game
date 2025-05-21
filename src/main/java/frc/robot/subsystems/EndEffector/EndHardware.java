// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Add your docs here. */
public class EndHardware implements EndBase {

    SparkMax indexMotor; 
    DigitalInput frontSensor, backSensor;

    public EndHardware() {
        // Constructor code here
        indexMotor = new SparkMax(Constants.EndEffector.ROLLER_MOTOR_ID, MotorType.kBrushless);

        frontSensor = new DigitalInput(Constants.EndEffector.FRONT_PIECE_SENSOR_ID);
        backSensor = new DigitalInput(Constants.EndEffector.BACK_PIECE_SENSOR_ID);

        configureMotors();
    }

    private void configureMotors(){
    SparkBaseConfig indexConfig = new SparkMaxConfig();
    // Roller Configs
    indexConfig.idleMode(IdleMode.kCoast);
    indexConfig.inverted(false);
    indexConfig.disableFollowerMode(); 
    // indexConfig.secondaryCurrentLimit(30);
    indexConfig.smartCurrentLimit(30);
    indexConfig.voltageCompensation(12.00);
    // ResetSafeParameters subject to change; not well documented 
    indexMotor.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

    @Override
    public void setSpeed(double speed) {
        indexMotor.set(speed);
    }

    @Override
    public boolean hasCoralFront() {
        return frontSensor.get();
    }

    @Override
    public boolean hasCoralBack() {
        return backSensor.get(); 
    }

    @Override
    public void update(EndBaseInputs inputs) {
        // Update the end effector state based on inputs
        inputs.hasCoralFront = hasCoralFront();
        inputs.hasCoralBack = hasCoralBack();
    }
}
