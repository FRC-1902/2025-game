// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.Position;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax leftMotor, rightMotor;
  private SparkBaseConfig configOne, configTwo;
  private DigitalInput limitSwitch;
  private PIDController pid;
  private Position targetPosition;
  private double targetHeight;
  private Alert alert;

  /** Creates a new Elevator. */
  public ElevatorSubsystem() {
    leftMotor = new SparkMax(Constants.Elevator.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(Constants.Elevator.RIGHT_MOTOR_ID, MotorType.kBrushless);

    configOne = new SparkMaxConfig();
    configTwo = new SparkMaxConfig();

    configureMotors();

    limitSwitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_PORT);

    pid = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);
    pid.setTolerance(Constants.Elevator.TOLERANCE);

    alert = new Alert(
        "Elevator/Bad Starting Pos, Robot Knows not where you are, and why you've done this.",
        AlertType.kError);

    if (!limitSwitchTriggered()) {
      alert.set(true);
      targetPosition = Position.STOP;
    }
  }

  private void configureMotors() {
    configOne.idleMode(IdleMode.kBrake);
    configOne.smartCurrentLimit(50);
    configOne.inverted(true); // todo: switch inverted
    configOne.voltageCompensation(12);

    configTwo.idleMode(IdleMode.kBrake);
    configTwo.smartCurrentLimit(50);
    configTwo.inverted(false); // todo: switch inverted
    configTwo.voltageCompensation(12);
    configTwo.follow(Constants.Elevator.LEFT_MOTOR_ID);
  }

  /**
   * 
   * @returns the current position of elevator
   */
  public double getPosition() {
    return (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getPosition()) * 0.5;
  }

  /**
   * 
   * @param targetPosition
   */
  public void setPosition(Position targetPosition) {
    this.targetPosition = targetPosition;
  }

  /**
   * 
   * @param targetHeight
   */
  public void setPosition(double targetHeight) {
    this.targetHeight = targetPosition.getHeight();
  }

  /**
   * 
   * @returns whether the limit switch was triggered
   */
  public boolean limitSwitchTriggered() {
    return limitSwitch.get();
  }

  /**
   * resets the pid
   */
  public void resetPID() {
    pid.reset();
  }

  /**
   * 
   * @returns whether or not the pid is at setpoint
   */
  public boolean pidAtSetpoint() {
    return pid.atSetpoint();
  }

  @Override
  public void periodic() {

    double power;
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Elevator/Limit Switch", limitSwitchTriggered());
    SmartDashboard.putString("Elevator/Current Target Height", targetPosition.getName());
    SmartDashboard.putNumber("ELevator/Current Position", getPosition());

    switch (targetPosition) {
      case L1:
        power = pid.calculate(getPosition(), targetHeight) + Constants.Elevator.kF;
        leftMotor.set(power);
        return;
      case L2:
        power = pid.calculate(getPosition(), targetHeight) + Constants.Elevator.kF;
        leftMotor.set(power);
        return;
      case L3:
        power = pid.calculate(getPosition(), targetHeight) + Constants.Elevator.kF;
        leftMotor.set(power);
        return;
      case STOP:
        leftMotor.set(0);
        return;
      case CLIMB:
        power = pid.calculate(getPosition(), targetHeight) + Constants.Elevator.kCLimb;
        while (!limitSwitchTriggered()) {
          leftMotor.set(power);
        }
    }
  }
}
