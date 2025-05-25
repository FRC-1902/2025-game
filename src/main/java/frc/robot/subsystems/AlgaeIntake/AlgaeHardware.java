// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import javax.sound.sampled.BooleanControl;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/** Add your docs here. */
public class AlgaeHardware implements AlgaeBase {

    SparkMax pivotMotor, rollerMotor;
    DigitalInput pSensor; 
    PIDController pid; 

    public AlgaeHardware(){
        pivotMotor = new SparkMax(Constants.AlgaeIntake.PIVOT_MOTOR_ID, MotorType.kBrushless);
        rollerMotor = new SparkMax(Constants.AlgaeIntake.ROLLER_MOTOR_ID, MotorType.kBrushless);  
        pSensor = new DigitalInput(Constants.AlgaeIntake.PIECE_SENSOR_ID); 

        configureMotors();

        pid = new PIDController(0, 0, 0); 
        pid.enableContinuousInput(0, 360); 
        pid.setTolerance(Constants.AlgaeIntake.TOLERANCE.getDegrees());

        resetPID();
    }

    private void configureMotors() {
        SparkBaseConfig pivotConfig = new SparkMaxConfig();
        SparkBaseConfig rollerConfig = new SparkMaxConfig();
        AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();

        // Pivot Configs
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.inverted(false); // todo: finish inverted
        pivotConfig.disableFollowerMode();
        // pivotConfig.secondaryCurrentLimit(40);
        pivotConfig.smartCurrentLimit(40);
        pivotConfig.voltageCompensation(12.00);

        // Roller Configs
        rollerConfig.idleMode(IdleMode.kBrake);
        rollerConfig.inverted(true); // todo: finish inverted
        rollerConfig.disableFollowerMode();
        // rollerConfig.secondaryCurrentLimit(30);
        rollerConfig.smartCurrentLimit(30);
        rollerConfig.voltageCompensation(12.00);

        // Encoder Config
        encoderConfig.zeroOffset(Constants.AlgaeIntake.ENCODER_OFFSET.getRotations());
        pivotConfig.apply(encoderConfig);

        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void powerCalc() {
        double power = pid.calculate(getAngle().getDegrees()) + Constants.AlgaeIntake.kG * Math.cos(getAngle().getRadians());
        pivotMotor.set(power);
    }

    @Override
    public void update(AlgaeBaseInputs inputs) {
        inputs.hasAlgae = pSensor.get();
        inputs.atSetpoint = pid.atSetpoint(); 
        powerCalc();
    }

    @Override
    public void setSpeed(double speed) {
        rollerMotor.set(speed);
    }

    @Override
    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(pivotMotor.get()); 
    }

    @Override
    public void setAngle(Rotation2d angle){
        pid.setSetpoint(angle.getDegrees());
    }

    @Override
    public boolean hasAlgae(){
       return pSensor.get();
    }

    @Override
    public void resetPID(){
        pid.reset();
    }
}