package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkMax;

import javax.sound.sampled.DataLine;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;

public class FloorIntakeSubsystem extends SubsystemBase {
  private SparkMax rollerMotor;
  private SparkMax pivotMotor;
  private DigitalInput irSensor;
  private PIDController pid;
  private Alert pivotAlert;
  private final ElevatorSubsystem elevatorSubsystem;
  private Watchdog pivotWatchdog; 
  private Pose3d intakePose;

  /** Creates a new FloorIntake. */
  public FloorIntakeSubsystem(ElevatorSubsystem elevatorSubsystem) {
    rollerMotor = new SparkMax(Constants.FloorIntake.ROLLER_MOTOR_ID, MotorType.kBrushless);
    pivotMotor = new SparkMax(Constants.FloorIntake.PIVOT_MOTOR_ID, MotorType.kBrushless);

    irSensor = new DigitalInput(Constants.FloorIntake.IR_SENSOR_ID);

    pid = new PIDController(Constants.FloorIntake.PIVOT_P, Constants.FloorIntake.PIVOT_I, Constants.FloorIntake.PIVOT_D);
    pid.disableContinuousInput();
    pid.setTolerance(Constants.FloorIntake.TOLERANCE.getDegrees());

    pivotAlert = new Alert("Pivot out of bounds", AlertType.kWarning);

    pivotWatchdog = new Watchdog(Constants.FloorIntake.MIN_PIVOT.getDegrees(), Constants.FloorIntake.MAX_PIVOT.getDegrees(), () -> getAngle().getDegrees());

    this.elevatorSubsystem = elevatorSubsystem;

    setAngle(Rotation2d.fromDegrees(5));
    
    // TODO: Check that motors aren't supposed to be inverted
    configureMotors();
  }

  private void configureMotors() {
    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
    SparkBaseConfig pivotConfig = new SparkMaxConfig();
    SparkBaseConfig rollerConfig = new SparkMaxConfig();

    // Pivot configs
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.inverted(true);
    pivotConfig.disableFollowerMode();
    // pivotConfig.secondaryCurrentLimit(40);
    pivotConfig.smartCurrentLimit(40);
    pivotConfig.voltageCompensation(12.00);

    // Roller configs
    rollerConfig.idleMode(IdleMode.kCoast);
    rollerConfig.inverted(true);
    rollerConfig.disableFollowerMode(); 
    // rollerConfig.secondaryCurrentLimit(30);
    rollerConfig.smartCurrentLimit(50);
    rollerConfig.voltageCompensation(12.00);

    // Encoder Configs 
    encoderConfig.zeroOffset(Constants.FloorIntake.ENCODER_OFFSET.getRotations());
    rollerConfig.apply(encoderConfig);

    // resetSafeParameters might be an issue
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  /**
   * 
   * @returns current angle in Rotation2d
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(1 - rollerMotor.getAbsoluteEncoder().getPosition());
  }

  /**
   * 
   * @param targetAngle sets the pivot angle
   */
  public void setAngle(Rotation2d targetAngle) {
    if (pivotWatchdog.checkWatchdog(targetAngle.getDegrees()) ) {
      DataLogManager.log("Specified input out of bounds on FloorIntake");
      return;
    }
    if(targetAngle.getDegrees() < 360 || targetAngle.getDegrees() > 350){
      pid.setSetpoint(0);
    }
    pid.setSetpoint(targetAngle.getDegrees());
    SmartDashboard.putNumber("FloorIntake/targetAngle", targetAngle.getDegrees());
  }

  /**
   * 
   * @param targetSpeed sets the speed of the roller motors between 1 and -1
   */
  public void setSpeed(double targetSpeed) {
    rollerMotor.set(targetSpeed); // TODO: Re-Enable
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
    return !irSensor.get();
  }

  // checks that the pivot isn't going out of tolerance, will send an alert if it
  // does
  private boolean pivotWatchdog() {
    if (pivotWatchdog.checkWatchdog()) {
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
    SmartDashboard.putData("PID/FloorIntake", pid); // TODO: Remove after tuning

    SmartDashboard.putBoolean("FloorIntake/atSetpoint", pid.atSetpoint());


    if(!elevatorSubsystem.isAtPosition(Constants.Elevator.Position.MIN) && getAngle().getDegrees() < 60){
      // DataLogManager.log("Elevator spooky in relation to floor intake");
      //setAngle(Rotation2d.fromDegrees(70)); // TODO: Re-Enable
    }
    
    double power = pid.calculate(getAngle().getDegrees())
        + Constants.FloorIntake.PIVOT_G * Math.cos(getAngle().getRadians() + Rotation2d.fromDegrees(5).getRadians());

    intakePose = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, getAngle().getDegrees(), 0)); // TODO: Math and offset

    SmartDashboard.putNumber("FloorIntake/Current Angle", getAngle().getDegrees());
    SmartDashboard.putBoolean("FloorIntake/Piece Sensor", pieceSensorActive());
    Logger.recordOutput("FloorIntake/Intake Pose", intakePose);

    if (pivotWatchdog()) {
      pivotMotor.set(0); // TODO: Re-Enable
      resetPID();
      return;
    }

    SmartDashboard.putNumber("FloorIntake/power", power);

    pivotMotor.set(power); // TODO: Re-Enable
  }
}
