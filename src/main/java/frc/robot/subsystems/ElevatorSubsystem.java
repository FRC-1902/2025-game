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
import edu.wpi.first.wpilibj.Servo;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax leftMotor, rightMotor;
  private SparkBaseConfig configOne, configTwo;
  private Servo servo;
  private DigitalInput limitSwitch;
  private PIDController pid;
  private Position targetPosition;
  private Alert badStart, boundsAlert;

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

    badStart = new Alert(
        "Elevator/Bad Starting Pos, Robot Knows not where you are, and why you've done this.",
        AlertType.kError);

    if (!limitSwitchTriggered()) {
      badStart.set(true);
    }

    boundsAlert = new Alert("Elevator/Elevator out of bounds", AlertType.kError);

    servo = new Servo(Constants.Elevator.SERVO_PORT); 
      
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

  /**
   * locks the servo from moving 
   */
  private void lockServo(){
    servo.set(0);
  }

  /**
   * Alerts if elevator is out of bounds
   */
  private void watchingDog(){
    if(getPosition() > Constants.Elevator.Position.MAX.getHeight() || getPosition() < Constants.Elevator.Position.MIN.getHeight()){
      boundsAlert.set(true);
    }
    else{
      boundsAlert.set(false);
    }
  }

  /**
   * full throttle downward for climb until limit switch is hit
   */
  private void climb(){
    if(!limitSwitchTriggered() && !pidAtSetpoint()){
      leftMotor.set(-1);
    }
    else{
      leftMotor.set(0);
      lockServo(); 
    } 
  }

  @Override
  public void periodic() {

    double power;
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Elevator/Limit Switch", limitSwitchTriggered());
    SmartDashboard.putNumber("ELevator/Current Position", getPosition());

      switch (targetPosition) {
        default:
          power = pid.calculate(getPosition(), targetPosition.getHeight()) + Constants.Elevator.kF;
          leftMotor.set(power);
          return;
        case CLIMB:
          climb(); 
      }
      watchingDog();
  }
}
