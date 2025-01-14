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
import edu.wpi.first.math.controller.PIDController;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax leftMotor, rightMotor;
  private SparkBaseConfig configOne, configTwo;
  private DigitalInput limitSwitch;
  private PIDController pid;
  private double targetPosition;

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
  public void setPosition(double targetPosition) {
    this.targetPosition = targetPosition;
  }

  /**
   * 
   * @returns whether the limit switch was triggered
   */
  public boolean limitSwitchTriggered() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double power = pid.calculate(getPosition(), targetPosition) + Constants.Elevator.kF;

    if (limitSwitchTriggered()) {
      leftMotor.set(0);
    } else {
      leftMotor.set(power);
    }
    SmartDashboard.putBoolean("Elevator/Limit Switch", limitSwitchTriggered());
  }
}
