// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FloorIntake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Add your docs here. */
public class FloorHardware implements FloorBase  {

    SparkMax pivotMotor, rollerMotor; 
    DigitalInput pSensor;
    PIDController pid;

    public FloorHardware() {
        // Constructor code here
        pivotMotor = new SparkMax(Constants.FloorIntake.PIVOT_MOTOR_ID, MotorType.kBrushless);
        rollerMotor = new SparkMax(Constants.FloorIntake.ROLLER_MOTOR_ID, MotorType.kBrushless);
        pSensor = new DigitalInput(Constants.FloorIntake.PIECE_SENSOR_ID);
        configureMotors();
        resetPID();
    }

    private void configureMotors() {

        AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
        SparkBaseConfig pivotConfig = new SparkMaxConfig();
        SparkBaseConfig rollerConfig = new SparkMaxConfig();

        // Pivot configs
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.inverted(true);
        pivotConfig.disableFollowerMode();
        // pivotConfig.secondaryCurrentLimit(40);
        pivotConfig.smartCurrentLimit(40);
        pivotConfig.voltageCompensation(12.00);

        // Roller configs
        rollerConfig.idleMode(IdleMode.kCoast);
        rollerConfig.inverted(true);
        rollerConfig.disableFollowerMode();
        // rollerConfig.secondaryCurrentLimit(30);
        rollerConfig.smartCurrentLimit(50);
        rollerConfig.voltageCompensation(12.00);

        // Encoder Configs
        encoderConfig.zeroOffset(Constants.FloorIntake.ENCODER_OFFSET.getRotations());
        rollerConfig.apply(encoderConfig);

        // resetSafeParameters might be an issue
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Motor configuration code here
        pid = new PIDController(0, 0, 0);
        pid.disableContinuousInput();
        pid.setTolerance(Constants.FloorIntake.TOLERANCE.getDegrees());
        pid.setIZone(10);
    }

    private void powerCalc(){
        double power = pid.calculate(getAngle().getDegrees())
        + Constants.FloorIntake.PIVOT_G * Math.cos(getAngle().getRadians() + Rotation2d.fromDegrees(4).getRadians());

        // compensate for friction if not near tolerance
        if (Math.abs(pid.getSetpoint() - getAngle().getDegrees()) > Constants.FloorIntake.TOLERANCE.getDegrees() / 2)
        power += Constants.FloorIntake.PIVOT_F * Math.signum(pid.getSetpoint() - getAngle().getDegrees());

        pivotMotor.set(power);
    }

    @Override
    public void setSpeed(double speed){
        rollerMotor.set(speed);
    };

    @Override
    public Rotation2d getAngle(){
        double angle = 1 - rollerMotor.getAbsoluteEncoder().getPosition();
        if (angle > 0.97) {  
          angle = 0;
        }
        return Rotation2d.fromRotations(angle);
    };

    @Override
    public void setAngle(Rotation2d angle){
        pid.setSetpoint(angle.getDegrees());
    };

    @Override
    public boolean hasCoral(){
        return pSensor.get();
    };

    @Override
    public void resetPID(){
        pid.reset();
    };

    @Override
    public void update(FloorBaseInputs inputs){
        inputs.atSetpoint = pid.atSetpoint();
        inputs.hasCoral = pSensor.get();
        powerCalc();
    };
}
