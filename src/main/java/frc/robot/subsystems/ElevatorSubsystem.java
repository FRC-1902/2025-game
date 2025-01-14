// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax leftMotor, rightMotor;
  private SparkBaseConfig configOne, configTwo;
  private DigitalInput leftLimitSwitch, rightLimitSwitch;
  private PIDController pid;
  private ElevatorFeedforward elevatorFeedforward; // subject for removal
  private Rotation2d targetAngle;

  /** Creates a new Elevator. */
  public ElevatorSubsystem() {
    leftMotor = new SparkMax(Constants.Elevator.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(Constants.Elevator.RIGHT_MOTOR_ID, MotorType.kBrushless);

    configOne = new SparkMaxConfig();
    configTwo = new SparkMaxConfig();

    configureMotors();

    leftLimitSwitch = new DigitalInput(Constants.Elevator.LB_SWITCH_PORT);
    rightLimitSwitch = new DigitalInput(Constants.Elevator.RB_SWITCH_PORT);

    pid = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);
    pid.setTolerance(Constants.Elevator.TOLERANCE);
    pid.enableContinuousInput(0, 360);

    elevatorFeedforward = new ElevatorFeedforward(Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV);
  }

  private void configureMotors() {
    configOne.idleMode(IdleMode.kBrake);
    configOne.smartCurrentLimit(30);
    configOne.inverted(true); // todo: switch inverted
    configOne.voltageCompensation(12);

    configTwo.idleMode(IdleMode.kBrake);
    configTwo.smartCurrentLimit(30);
    configTwo.inverted(false); // todo: switch inverted
    configTwo.voltageCompensation(12);
    configTwo.follow(Constants.Elevator.LEFT_MOTOR_ID);

  }

  /**
   * 
   * @returns the current angle in Rotation2d's
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees((leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getPosition()) * 0.5);
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
   * @returns whether the limit switch was triggered
   */
  public boolean limitSwitchTriggered() {
    if (leftLimitSwitch.get() || rightLimitSwitch.get()) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double power = pid.calculate(getAngle().getDegrees(), targetAngle.getDegrees()) + Constants.Elevator.kF;

    if (limitSwitchTriggered()) {
      leftMotor.set(0);
    } else {
      leftMotor.set(power);
    }
    SmartDashboard.putBoolean("Limit Switch Triggered", limitSwitchTriggered());
    SmartDashboard.putNumber("Current Angle", getAngle().getDegrees());
  }
}
