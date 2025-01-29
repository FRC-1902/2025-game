// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.Position;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Servo;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

// WPILib color utilities
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax leftMotor, rightMotor;
  private Servo servo;
  private DigitalInput limitSwitch;
  private PIDController pid;
  private Position targetPosition;
  private Alert badStart, boundsAlert, servoAlert;

    // ------------------ Logged Mechanism Objects ------------------
    private final LoggedMechanism2d elevatorMech;         // The overall canvas
    private final LoggedMechanismRoot2d elevatorRoot;     // The anchor point
    private final LoggedMechanismLigament2d towerLigament;  // Represents the fixed tower
    private final LoggedMechanismLigament2d carriageLigament; 

  /** Creates a new Elevator. */
  public ElevatorSubsystem() {
    leftMotor = new SparkMax(Constants.Elevator.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(Constants.Elevator.RIGHT_MOTOR_ID, MotorType.kBrushless);

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

    servoAlert = new Alert("Elevator/Cannot Exit Climb, Servo is locked", AlertType.kWarning);

    elevatorMech = new LoggedMechanism2d(2, 2); // 2x2 "canvas" for the elevator
    // Place root near bottom-left (for a nice display). You can tweak this.
    elevatorRoot = elevatorMech.getRoot("ElevatorRoot", 0.5, 0.0);

    // A fixed vertical tower (e.g., 1.5 "units" tall, angle = 90° to go up).
    towerLigament = elevatorRoot.append(
      new LoggedMechanismLigament2d(
        "ElevatorTower",
        1.5, // length in Mechanism2d "units"
        90,  // angle in degrees (0 is right, 90 is straight up)
        6,   // line width
        new Color8Bit(Color.kGray)
      )
    );
        // A carriage that moves up/down along the tower.
    // We'll represent it as a second ligament that starts at length=0,
    // but changes each loop to match the elevator's position.
    carriageLigament = elevatorRoot.append(
      new LoggedMechanismLigament2d(
        "Carriage",
        0.0,          // initial length
        90.0,         // angle (vertical)
        8,            // line width
        new Color8Bit(Color.kYellow)
      )
    );
  }

  private void configureMotors() {
    SparkBaseConfig configOne = new SparkMaxConfig();
    SparkBaseConfig configTwo = new SparkMaxConfig();
    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();

    configOne.idleMode(IdleMode.kBrake);
    configOne.smartCurrentLimit(50);
    configOne.inverted(true); // todo: switch inverted
    configOne.voltageCompensation(12);

    configTwo.idleMode(IdleMode.kBrake);
    configTwo.smartCurrentLimit(50);
    configTwo.inverted(false); // todo: switch inverted
    configTwo.voltageCompensation(12);
    configTwo.follow(Constants.Elevator.LEFT_MOTOR_ID);

    encoderConfig.positionConversionFactor(Constants.Elevator.CONVERSION_FACTOR);

    configOne.apply(encoderConfig); 
    configTwo.apply(encoderConfig); 
    // ResetSafeParameters not well documented 
    leftMotor.configure(configOne, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); 
    rightMotor.configure(configTwo, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); 
  }

  /**
   * 
   * @returns the current position of elevator in meters
   */
  public double getPosition() {
    return (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getPosition()) * 0.5;
  }

  /**
   * 
   * @param targetPosition
   */
  public void setPosition(Position targetPosition) {
    if(targetPosition != Constants.Elevator.Position.CLIMB){
      servoAlert.set(true);
    }
    else{
      this.targetPosition = targetPosition;
    }
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
  // public void setLocked(boolean lock) {
  //   if (lock) {
  //     servo.setAngle(Constants.Elevator.LOCK_ANGLE.getDegrees());
  //   } else {
  //     servo.setAngle(Constants.Elevator.UNLOCK_ANGLE.getDegrees());
  //   }
  // }

  // /**
  //  * 
  //  * @returns if the servo is at the locked angle or not
  //  */
  // public boolean isLocked(){
  //   return 0.001 < Math.abs(servo.getAngle() - Constants.Elevator.LOCK_ANGLE.getDegrees());
  // }

  /**
   * Alerts if elevator is out of bounds
   */
  private boolean watchingDog() {
    if (getPosition() > Constants.Elevator.Position.MAX.getHeight()
        || getPosition() < Constants.Elevator.Position.MIN.getHeight()) {
      boundsAlert.set(true);
      return true;
    } else {
      boundsAlert.set(false);
      return false;
    }
  }

  /**
   * full throttle downward for climb until limit switch is hit
   */
  private void climb() {
    if (!limitSwitchTriggered()) {
      leftMotor.set(-1);
    } else {
      leftMotor.set(0);
    }
  }

  @Override
  public void periodic() {

    double power;
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Elevator/Limit Switch", limitSwitchTriggered());
    SmartDashboard.putNumber("ELevator/Current Position", getPosition());
    SmartDashboard.putNumber("Elevator/Servo Position", servo.getPosition());

    // Update the elevator's carriage ligament length to match the real position
    // We assume getPosition() is in the 0-1.5m range, for example. You can scale if needed.
    carriageLigament.setLength(getPosition());

    // if (watchingDog() || isLocked()) {
    //   leftMotor.set(0);
    //   Logger.recordOutput("Mechanism/Elevator", elevatorMech);
    //   return; 
    // }

    switch (targetPosition) {
      case CLIMB:
        climb();
        Logger.recordOutput("Mechanism/Elevator", elevatorMech);
        return;
      default:
        power = pid.calculate(getPosition(), targetPosition.getHeight()) + Constants.Elevator.kF;
        leftMotor.set(power);
        Logger.recordOutput("Mechanism/Elevator", elevatorMech);
        return;
    }
  }
}
