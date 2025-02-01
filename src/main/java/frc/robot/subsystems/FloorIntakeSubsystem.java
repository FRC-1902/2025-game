// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// AdvantageKit mechanism imports
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// WPILib color utilities
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FloorIntakeSubsystem extends SubsystemBase {
  private SparkMax rollerMotor;
  private SparkMax pivotMotor;
  private DigitalInput irSensor;
  private PIDController pid;
  private Alert pivotAlert;
  private final ElevatorSubsystem elevatorSubsystem;

    // ------------------ Logged Mechanism for AdvantageKit ------------------
    private final LoggedMechanism2d intakeMech;         // The Mechanism2d canvas
    private final LoggedMechanismRoot2d pivotRoot;      // The pivot's anchor point
    private final LoggedMechanismLigament2d intakeBar; 

  /** Creates a new FloorIntake. */
  public FloorIntakeSubsystem(ElevatorSubsystem elevatorSubsystem) {
    rollerMotor = new SparkMax(Constants.FloorIntake.ROLLERMOTOR_PORT, MotorType.kBrushless);

    pivotMotor = new SparkMax(Constants.FloorIntake.PIVOTMOTOR_PORT, MotorType.kBrushless);

    pid = new PIDController(
    Constants.FloorIntake.PIVOT_P,
    Constants.FloorIntake.PIVOT_I,
    Constants.FloorIntake.PIVOT_D);
    pid.enableContinuousInput(0, 360);
    pid.setTolerance(Constants.FloorIntake.TOLERANCE.getDegrees());

    irSensor = new DigitalInput(Constants.FloorIntake.IR_SENSOR_PORT);

    pivotAlert = new Alert("Pivot out of bounds", AlertType.kWarning);
    // Check that motors aren't supposed to be inverted
    configureMotors();

    this.elevatorSubsystem = elevatorSubsystem;

    // ------------------ Create Logged Mechanism2d ------------------
    // 3x3 "canvas" or whatever size you prefer
    intakeMech = new LoggedMechanism2d(3, 3);
    // Root at (1.5, 1.5) so that the bar can rotate around center
    pivotRoot = intakeMech.getRoot("FloorIntakePivot", 1.5, 1.5);

    // A single bar representing the intake pivot
    // 1.0 length, 0° initial angle, color orange, etc.
    intakeBar = pivotRoot.append(
        new LoggedMechanismLigament2d(
            "IntakeBar",
            1.0,                  // length
            0.0,                  // initial angle
            6,                    // line width
            new Color8Bit(Color.kOrange)
        )
    );
  }

  private void configureMotors() {
    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
    SparkBaseConfig pivotConfig = new SparkMaxConfig();
    SparkBaseConfig rollerConfig = new SparkMaxConfig();

    // Pivot configs
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.inverted(false);
    pivotConfig.disableFollowerMode();
    pivotConfig.secondaryCurrentLimit(30);
    pivotConfig.smartCurrentLimit(30);
    pivotConfig.voltageCompensation(12.00);

    // Roller configs
    rollerConfig.idleMode(IdleMode.kCoast);
    rollerConfig.inverted(false);
    rollerConfig.disableFollowerMode(); 
    rollerConfig.secondaryCurrentLimit(30);
    rollerConfig.smartCurrentLimit(30);
    rollerConfig.voltageCompensation(12.00);

    // Encoder Configs 
    encoderConfig.zeroOffset(Constants.FloorIntake.ENCODER_OFFSET.getRotations());
    pivotConfig.apply(encoderConfig);

    // resetSafeParameters might be an issue
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * 
   * @returns current angle in Rotation2d
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(pivotMotor.getAbsoluteEncoder().getPosition());
  }

  /**
   * 
   * @param targetAngle sets the pivot angle
   */
  public void setAngle(Rotation2d targetAngle) {
    pid.setSetpoint(targetAngle.getDegrees());
    System.out.println("[FloorIntakeSubsystem] setAngle(" + targetAngle.getDegrees() + " deg)");
    Logger.recordOutput("FloorIntake/SetAngle", targetAngle.getDegrees());
  }

  /**
   * 
   * @param targetSpeed sets the speed of the roller motors between 1 and -1
   */
  public void setSpeed(double targetSpeed) {
    rollerMotor.set(targetSpeed);
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
    // if (getAngle().getDegrees() >= Constants.FloorIntake.MAX_PIVOT.getDegrees() || 
    //     getAngle().getDegrees() <= Constants.FloorIntake.MIN_PIVOT.getDegrees()
    //   ) {
    //   pivotAlert.set(true);
    //   return true;
    // } else {
    //   pivotAlert.set(false);
    //   return false;
    // }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!elevatorSubsystem.isAtPosition(Constants.Elevator.Position.MIN) && getAngle().getDegrees() < 60){
      DataLogManager.log("Elevator spooky in relation to floor intake");
      setAngle(Rotation2d.fromDegrees(61));
    }
    
    double power = pid.calculate(getAngle().getDegrees())
        + Constants.FloorIntake.PIVOT_G * Math.cos(getAngle().getRadians());

    SmartDashboard.putNumber("Floor-Intake/Current Angle", getAngle().getDegrees());

    if (pivotWatchdog()) {
      pivotMotor.set(0);
      resetPID();
      Logger.recordOutput("Mechanism/FloorIntake", intakeMech);
      System.out.println("[FloorIntakeSubsystem] pivotWatchdog triggered => motor=0");
      return;
    }

    SmartDashboard.putNumber("FloorIntake/AngleDeg", getAngle().getDegrees());
    Logger.recordOutput("FloorIntake/AngleDeg", getAngle().getDegrees());
    Logger.recordOutput("FloorIntake/PIDSetpoint", pid.getSetpoint());
    Logger.recordOutput("FloorIntake/Power", power);

    pivotMotor.set(power);
    intakeBar.setAngle(getAngle().getDegrees() - 90.0);
    Logger.recordOutput("Mechanism/FloorIntake", intakeMech);
  }
}
