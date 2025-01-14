// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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

public class EndAffectorSubsystem extends SubsystemBase {
  private SparkMax rollerMotor, pivotMotor;
  private SparkBaseConfig rollerConfig, pivotConfig;
  private PIDController pid;
  private Rotation2d targetAngle;
  private double targetSpeed;
  private Alert alert;

  /** Creates a new EndAffectorSubsystem. */
  public EndAffectorSubsystem() {
    rollerMotor = new SparkMax(Constants.EndAffector.ROLLER_ID, MotorType.kBrushless);
    pivotMotor = new SparkMax(Constants.EndAffector.PIVOT_ID, MotorType.kBrushless);

    rollerConfig = new SparkMaxConfig();
    pivotConfig = new SparkMaxConfig();

    pid = new PIDController(Constants.EndAffector.kP, Constants.EndAffector.kI, Constants.EndAffector.kD);
    pid.enableContinuousInput(0, 360);
    pid.setTolerance(Constants.EndAffector.TOLERANCE.getDegrees());
    configureMotors();

    alert = new Alert("Algae Pivot Out Of Bounds", AlertType.kWarning);
  }

  private void configureMotors() {
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

    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * 
   * @returns current angle in Rotation2d's
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(pivotMotor.getAbsoluteEncoder().getPosition());
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
   * @returns current speed of rollers
   */
  public double getSpeed() {
    return rollerMotor.getEncoder().getVelocity();
  }

  /**
   * 
   * @param targetSpeed
   */
  public void setSpeed(double targetSpeed) {
    this.targetSpeed = targetSpeed;
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
    if (getAngle().getDegrees() >= Constants.EndAffector.MAX_PIVOT.getDegrees()
        || getAngle().getDegrees() <= Constants.EndAffector.MIN_PIVOT.getDegrees()) {
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
    double power = pid.calculate(getAngle().getDegrees(), targetAngle.getDegrees())
        + Constants.EndAffector.kG * Math.cos(getAngle().getRadians());

    SmartDashboard.putNumber("Algae Intake Current Angle ", getAngle().getDegrees());

    if (pivotWatchdog()) {
      pivotMotor.set(0);
      resetPID();
      return;
    }
    pivotMotor.set(power);
  }
}
