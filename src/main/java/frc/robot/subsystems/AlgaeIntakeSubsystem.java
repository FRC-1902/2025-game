package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  private SparkMax rollerMotor;
  private SparkMax pivotMotor;
  private PIDController pid;
  private Alert alert;
  private DigitalInput irSensor; 
  private Watchdog pivotWatchdog;
  private final ElevatorSubsystem elevatorSubsystem;

  /** Creates a new AlgaeIntakeSubsystem. */
  public AlgaeIntakeSubsystem(ElevatorSubsystem elevatorSubsystem) {
    rollerMotor = new SparkMax(Constants.AlgaeIntake.ROLLER_MOTOR_ID, MotorType.kBrushless);
    pivotMotor = new SparkMax(Constants.AlgaeIntake.PIVOT_MOTOR_ID, MotorType.kBrushless);

    pid = new PIDController(Constants.AlgaeIntake.kP, Constants.AlgaeIntake.kI, Constants.AlgaeIntake.kD);
    pid.enableContinuousInput(0, 360); 
    pid.setTolerance(Constants.AlgaeIntake.TOLERANCE.getDegrees());
    configureMotors();

    alert = new Alert("Algae Pivot Out Of Bounds", AlertType.kWarning);

    irSensor = new DigitalInput(Constants.AlgaeIntake.IR_SENSOR_ID);

    pivotWatchdog = new Watchdog(Constants.AlgaeIntake.MAX_PIVOT.getDegrees(), Constants.AlgaeIntake.MIN_PIVOT.getDegrees(), () -> getAngle().getDegrees());

    this.elevatorSubsystem = elevatorSubsystem;

    setAngle(Constants.AlgaeIntake.DEFAULT_ANGLE); 
  }

  private void configureMotors() {
    SparkBaseConfig pivotConfig = new SparkMaxConfig();
    SparkBaseConfig rollerConfig = new SparkMaxConfig();
    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();

    // Pivot Configs
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.inverted(false); // todo: finish inverted
    pivotConfig.disableFollowerMode();
    // pivotConfig.secondaryCurrentLimit(40);
    pivotConfig.smartCurrentLimit(40);
    pivotConfig.voltageCompensation(12.00);

    // Roller Configs
    rollerConfig.idleMode(IdleMode.kBrake);
    rollerConfig.inverted(true); // todo: finish inverted
    rollerConfig.disableFollowerMode();
    // rollerConfig.secondaryCurrentLimit(30);
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
    if (pivotWatchdog.checkWatchdog(targetAngle.getDegrees())) {
      DataLogManager.log("Specified input out of bounds on AlgaeIntake");
      return;
    }
    pid.setSetpoint(targetAngle.getDegrees());
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
   * @returns if irSensor is triggered or not 
   */
  public boolean isAlgaeDetected(){
    return !irSensor.get(); 
  }

  /**
   * 
   * @returns whether or not pivot is out of bounds or not
   */
  private boolean pivotWatchdog() {
    if (pivotWatchdog.checkWatchdog()) {
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
    SmartDashboard.putData("PID/Algae", pid); 
    SmartDashboard.putBoolean("FloorIntake/WatchingDog", pivotWatchdog());

    double power = pid.calculate(getAngle().getDegrees())
      + Constants.AlgaeIntake.kG * Math.cos(getAngle().getRadians());

    Pose3d intakePose = new Pose3d(new Translation3d(0.312, 0 + elevatorSubsystem.getPosition(), 0.4), new Rotation3d(0,0,0)); // TODO: Offset and Math

    SmartDashboard.putNumber("AlgaeIntake/Pivot Angle ", getAngle().getDegrees());
    SmartDashboard.putBoolean("AlgaeIntake/Algae Detected ", isAlgaeDetected());
    Logger.recordOutput("AlgaeIntake/Intake Pose", intakePose);
    
    if (pivotWatchdog()) {
      pivotMotor.set(0); 
      resetPID();
      return;
    }
    pivotMotor.set(power); 
  }
}
