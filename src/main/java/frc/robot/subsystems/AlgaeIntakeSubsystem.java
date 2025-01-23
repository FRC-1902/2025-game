// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  private SparkMax rollerMotor, pivotMotor;
  private PIDController pid;
  private Rotation2d targetAngle;
  private Alert alert;
  private Watchdog pivotWatchdog; 
  private DoubleSupplier current;

  /** Creates a new AlgaeIntakeSubsystem. */
  public AlgaeIntakeSubsystem() {
    rollerMotor = new SparkMax(Constants.AlgaeIntake.ROLLER_ID, MotorType.kBrushless);
    pivotMotor = new SparkMax(Constants.AlgaeIntake.PIVOT_ID, MotorType.kBrushless);

    pid = new PIDController(Constants.AlgaeIntake.kP, Constants.AlgaeIntake.kI, Constants.AlgaeIntake.kD);
    pid.enableContinuousInput(0, 360);
    pid.setTolerance(Constants.AlgaeIntake.TOLERANCE.getDegrees());
    configureMotors();

    alert = new Alert("Algae Pivot Out Of Bounds", AlertType.kWarning);

    pivotWatchdog = new Watchdog(Constants.AlgaeIntake.MIN_PIVOT.getDegrees(), Constants.AlgaeIntake.MAX_PIVOT.getDegrees(), current);
  }

  private void configureMotors() {
    SparkBaseConfig pivotConfig = new SparkMaxConfig();
    SparkBaseConfig rollerConfig = new SparkMaxConfig();
    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();

    // Pivot Configs
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.inverted(false); // todo: finish inverted
    pivotConfig.disableFollowerMode();
    pivotConfig.secondaryCurrentLimit(30);
    pivotConfig.smartCurrentLimit(30);
    pivotConfig.voltageCompensation(12.00);

    // Roller Configs
    rollerConfig.idleMode(IdleMode.kBrake);
    rollerConfig.inverted(false); // todo: finish inverted
    rollerConfig.disableFollowerMode();
    rollerConfig.secondaryCurrentLimit(30);
    rollerConfig.smartCurrentLimit(30);
    rollerConfig.voltageCompensation(12.00);

    // Encoder Config 
    encoderConfig.zeroOffset(Constants.AlgaeIntake.ENCODER_OFFSET.getRotations());
    pivotConfig.apply(encoderConfig);

    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * 
   * @returns current angle in Rotation2d's. Impacts pivot. 
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(pivotMotor.getAbsoluteEncoder().getPosition());
  }

  /**
   * 
   * @param targetAngle
   */
  public void setAngle(Rotation2d targetAngle) {
    this.targetAngle = targetAngle;
  }

  /**
   * 
   * @param targetSpeed
   */
  public void setSpeed(double targetSpeed) {
    rollerMotor.set(targetSpeed);
  }

  /**
   * resets pid
   */
  public void resetPID() {
    pid.reset();
  }

  /**
   * 
   * @returns whether or not pivot is out of bounds or not
   */
  private boolean pivotWatchdog() {
    if (!pivotWatchdog.checkWatchingdog()) {
      alert.set(true);
      return true;
    } else {
      alert.set(false);
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    current = () -> getAngle().getDegrees();

    double power = pid.calculate(getAngle().getDegrees(), targetAngle.getDegrees())
        + Constants.AlgaeIntake.kG * Math.cos(getAngle().getRadians());

    SmartDashboard.putNumber("AlgaeIntake/Current Angle ", getAngle().getDegrees());

    if (pivotWatchdog()) {
      pivotMotor.set(0);
      resetPID();
      return;
    }
    pivotMotor.set(power);
  }
}
