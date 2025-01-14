// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;

public class FloorIntake extends SubsystemBase {
  private SparkMax rollerMotor1, rollerMotor2;
  private SparkMax pivotMotor;
  private SparkBaseConfig pivotConfig, rollerConfig1, rollerConfig2;
  private DigitalInput irSensor;
  private PIDController pid;
  private Alert pivotAlert;

  /** Creates a new FloorIntake. */
  public FloorIntake() {
    rollerMotor1 = new SparkMax(Constants.FloorIntake.ROLLERMOTOR1_PORT, MotorType.kBrushless);
    rollerMotor2 = new SparkMax(Constants.FloorIntake.ROLLERMOTOR2_PORT, MotorType.kBrushless);

    pivotMotor = new SparkMax(Constants.FloorIntake.PIVOTMOTOR1_PORT, MotorType.kBrushless);

    pivotConfig = new SparkMaxConfig();
    rollerConfig1 = new SparkMaxConfig();
    rollerConfig2 = new SparkMaxConfig();

    pid = new PIDController(Constants.FloorIntake.PIVOT_P, Constants.FloorIntake.PIVOT_I,
        Constants.FloorIntake.PIVOT_D);
    pid.enableContinuousInput(0, 360);
    pid.setTolerance(Constants.FloorIntake.TOLERANCE.getDegrees());

    irSensor = new DigitalInput(Constants.FloorIntake.IR_SENSOR_PORT);

    pivotAlert = new Alert("Pivot out of bounds", AlertType.kWarning);
    // Check that motors aren't supposed to be inverted
    configureMotors();
  }

  private void configureMotors() {
    // Pivot configs
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.inverted(false);
    pivotConfig.disableFollowerMode();
    pivotConfig.secondaryCurrentLimit(30);
    pivotConfig.smartCurrentLimit(30);
    pivotConfig.voltageCompensation(12.00);

    // Roller configs
    rollerConfig1.idleMode(IdleMode.kCoast);
    rollerConfig1.inverted(false);
    rollerConfig1.secondaryCurrentLimit(30);
    rollerConfig1.smartCurrentLimit(30);
    rollerConfig1.voltageCompensation(12.00);

    rollerConfig2.idleMode(IdleMode.kCoast);
    rollerConfig2.inverted(true); // check inversion
    rollerConfig2.follow(Constants.FloorIntake.PIVOTMOTOR1_PORT); // check follower mode
    rollerConfig2.secondaryCurrentLimit(30);
    rollerConfig2.smartCurrentLimit(30);
    rollerConfig2.voltageCompensation(12.00);
    // resetSafeParameters might be an issue
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    rollerMotor1.configure(rollerConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rollerMotor2.configure(rollerConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * 
   * @returns current angle in Rotation2d
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(pivotMotor.getAbsoluteEncoder().getPosition());
  }

  /**
   * 
   * @param targetAngle sets the pivot angle
   */
  public void setAngle(Rotation2d targetAngle) {
    pid.setSetpoint(targetAngle.getDegrees());
  }

  /**
   * 
   * @returns the average current speed of the roller motors
   */
  public double getSpeed() {
    return (rollerMotor1.getAbsoluteEncoder().getVelocity() + rollerMotor2.getAbsoluteEncoder().getVelocity()) * 0.5;
  }

  /**
   * 
   * @param targetSpeed sets the speed of the roller motors
   */
  public void setSpeed(double targetSpeed) {
    rollerMotor1.set(targetSpeed);
  }

  /**
   * 
   * @returns if the pid is at setpoint or not
   */
  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  // resets the pid
  public void resetPID() {
    pid.reset();
  }

  /**
   * 
   * @returns whether or not a piece is detected
   */
  public boolean pieceSensorActive() {
    return irSensor.get();
  }

  // checks that the pivot isn't going out of tolerance, will send an alert if it
  // does
  private boolean pivotWatchdog() {
    if (getAngle().getDegrees() >= Constants.FloorIntake.MAX_PIVOT.getDegrees()
        || getAngle().getDegrees() <= Constants.FloorIntake.MIN_PIVOT.getDegrees()) {
      pivotAlert.set(true);
      return true;
    } else {
      pivotAlert.set(false);
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double power = pid.calculate(getAngle().getDegrees())
        + Constants.FloorIntake.PIVOT_G * Math.cos(getAngle().getRadians());

    SmartDashboard.putNumber("Floor-Intake Current Angle ", getAngle().getDegrees());

    if (pivotWatchdog()) {
      pivotMotor.set(0);
      resetPID();
      return;
    }

    pivotMotor.set(power);
  }
}
