// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

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

/** Add your docs here. */
public class AlgaeHardware implements AlgaeBase {

    SparkMax pivotMotor, rollerMotor;
    DigitalInput pSensor; 
    PIDController pid; 

    public AlgaeHardware(){
        pivotMotor = new SparkMax(AlgaeConstants.MotorIDs.PIVOT_MOTOR_ID, MotorType.kBrushless);
        rollerMotor = new SparkMax(AlgaeConstants.MotorIDs.ROLLER_MOTOR_ID, MotorType.kBrushless);  
        pSensor = new DigitalInput(AlgaeConstants.MotorIDs.PIECE_SENSOR_ID); 

        configureMotors();

        pid = new PIDController(AlgaeConstants.PID.kP, AlgaeConstants.PID.kI, AlgaeConstants.PID.kD); 
        pid.enableContinuousInput(0, 360); 
        pid.setTolerance(AlgaeConstants.Offsets.TOLERANCE.getDegrees());

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
        encoderConfig.zeroOffset(AlgaeConstants.Offsets.ENCODER_OFFSET.getRotations());
        pivotConfig.apply(encoderConfig);

        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void powerCalc() {
        double power = pid.calculate(getAngle().getDegrees()) + AlgaeConstants.PID.kG * Math.cos(getAngle().getRadians());
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
        return Rotation2d.fromDegrees(pivotMotor.getAbsoluteEncoder().getPosition()); 
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
