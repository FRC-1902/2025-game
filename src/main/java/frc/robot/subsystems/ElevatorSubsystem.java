package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.Position;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkFlex leftMotor;
  private SparkFlex rightMotor;
  private Servo servo;
  private DigitalInput limitSwitch;
  private PIDController pid;
  private Position targetPosition;
  private Alert badStart;
  private Alert boundsAlert;
  private Alert servoAlert;
  private Watchdog elevatorWatchdog;
  private double unlockTime;

  /** Creates a new Elevator. */
  public ElevatorSubsystem() {
    leftMotor = new SparkFlex(Constants.Elevator.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkFlex(Constants.Elevator.RIGHT_MOTOR_ID, MotorType.kBrushless);
    servo = new Servo(Constants.Elevator.SERVO_ID);

    limitSwitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_ID);

    pid = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);
    pid.setTolerance(Constants.Elevator.TOLERANCE);

    badStart = new Alert(
      "Elevator start position wrong, limit switch not triggered",
      AlertType.kError
    );

    if (!limitSwitchTriggered()) {
      badStart.set(true);
    }

    boundsAlert = new Alert("Elevator/Elevator out of bounds", AlertType.kError);
    servoAlert = new Alert("Elevator/Cannot Exit Climb, Servo is locked", AlertType.kWarning);

    elevatorWatchdog = new Watchdog(Constants.Elevator.Position.MAX.getHeight() + 0.01, Constants.Elevator.Position.MIN.getHeight() - 0.05, this::getPosition);

    targetPosition = Constants.Elevator.Position.MIN;

    servo.setAngle(Constants.Elevator.UNLOCK_ANGLE);

    configureMotors();
  }

  private void configureMotors() {
    SparkBaseConfig configOne = new SparkMaxConfig();
    SparkBaseConfig configTwo = new SparkMaxConfig();

    configOne.idleMode(IdleMode.kCoast);
    configOne.smartCurrentLimit(50);
    configOne.inverted(true); // todo: switch inverted
    configOne.voltageCompensation(12);

    configTwo.idleMode(IdleMode.kCoast);
    configTwo.smartCurrentLimit(50);
    configTwo.inverted(false); // todo: switch inverted
    configTwo.voltageCompensation(12);

    configOne.encoder.positionConversionFactor(Constants.Elevator.CONVERSION_FACTOR);
    configTwo.encoder.positionConversionFactor(Constants.Elevator.CONVERSION_FACTOR);

    // ResetSafeParameters not well documented 
    leftMotor.configure(configOne, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); 
    rightMotor.configure(configTwo, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); 
  }

  /**
   * 
   * @returns the current position of elevator in meters
   */
  public double getPosition() {
    return leftMotor.getEncoder().getPosition();
  }

  /**
   * 
   * @param targetPosition
   */
  public void setPosition(Position targetPosition) {
    if (elevatorWatchdog.checkWatchdog(targetPosition.getHeight())) {
      DataLogManager.log("Specified input out of bounds on Elevator");
      return;
    }

    this.targetPosition = targetPosition;
    pid.setSetpoint(targetPosition.getHeight());
  }

  /**
   * 
   * @returns whether the limit switch was triggered
   */
  public boolean limitSwitchTriggered() {
    return !limitSwitch.get();
  }

  /**
   * resets the pid
   */
  public void resetPID() {
    pid.reset();
  }

  public boolean isAtPosition() {
    return isAtPosition(targetPosition);
  }

  /**
   * 
   * @param pos
   * @returns whether the elevator is at a tolerance to the specified position or not
   */
  public boolean isAtPosition(Position pos){
    return Math.abs(getPosition() - pos.getHeight()) <= Constants.Elevator.TOLERANCE;
  }

  /**
   * locks the servo from moving
   */
  public void setLocked(boolean lock) {
    if (lock) {
      servo.setAngle(Constants.Elevator.LOCK_ANGLE);
    } else {
      unlockTime = Timer.getFPGATimestamp();
      servo.setAngle(Constants.Elevator.UNLOCK_ANGLE);
    }
  }

  /**
   * 
   * @returns if the targetAngle is at the locked position or not
   */
  public boolean isLocked(){
    return 0.001 > Math.abs(servo.getAngle() - Constants.Elevator.LOCK_ANGLE);
  }

  /**
   * Alerts if elevator is out of bounds
   */
  private boolean watchDog() {
    if (elevatorWatchdog.checkWatchdog()) {
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
  private double climb() {
    double startTime = Timer.getFPGATimestamp();
    if (!limitSwitchTriggered() && !isLocked()) {
      return -0.5; // TODO: change speed
    } else {
      setLocked(true);
      if (Timer.getFPGATimestamp() - startTime > 1) {
        return 0;
      }
      return -0.2;
    }
  }

  /**
   * go to the bottom of the elevator to re-home
   */
  private double home() {
    if (!limitSwitchTriggered() && !isLocked()) {
      return -0.3; // TODO: change speed
    } else {
      return 0;
    }
  }

  /**
   * Checks if locked and if not, calculates the power to set the motors to
   * @returns the power to set the motors to
   */
  private double calcPID() {
    if (!isLocked() && Timer.getFPGATimestamp() - unlockTime > 0.1) {
      return pid.calculate(getPosition()) + Constants.Elevator.kF + Constants.Elevator.kS * Math.signum(pid.getSetpoint() - getPosition());
    } else {
      return 0;
    }    
  }

  private void publishTelemetry(){
    SmartDashboard.putData("PID/Elevator", pid); // TODO: Remove after tuning

    SmartDashboard.putBoolean("Elevator/Limit Switch", limitSwitchTriggered());
    SmartDashboard.putNumber("Elevator/Current Position", getPosition());
    SmartDashboard.putNumber("Elevator/Servo Position", servo.getAngle());
    SmartDashboard.putBoolean("Elevator/Elevator Locked", isLocked());
    SmartDashboard.putBoolean("Elevator/isAtPos", isAtPosition(targetPosition));
    SmartDashboard.putString("Elevator/Enum Position", targetPosition.name());

    SmartDashboard.putNumber("Elevator/Motors/Left Power", leftMotor.get());
    SmartDashboard.putNumber("Elevator/Motors/Right Power", rightMotor.get());
    SmartDashboard.putNumber("Elevator/Motors/Left Position", leftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator/Motors/Right Position", rightMotor.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    double power;

    // simulate elevator for 3d visualization
    Pose3d elevatorPose = new Pose3d(new Translation3d(0, 0, getPosition()/2), new Rotation3d()); // TODO: Math
    Pose3d carriagePose = new Pose3d(new Translation3d(0, 0, getPosition()*2), new Rotation3d()); // TODO: Math
    Logger.recordOutput("Elevator/Stage", elevatorPose);
    Logger.recordOutput("Elevator/Carriage", carriagePose);

    publishTelemetry();

    // zero out on ground
    // XXX: might want to remove b/c coral getting stuck
    // if (limitSwitchTriggered()) {
    //   leftMotor.getEncoder().setPosition(0);
    //   rightMotor.getEncoder().setPosition(0);
    // }

    if (watchDog()) {
      leftMotor.set(0);
      rightMotor.set(0);
      return; 
    }

    switch (targetPosition) {
      case HOLD:
        power = 0;
        break;
      case HOME:
        if (isLocked()) {
          setLocked(false);
        }
        power = home();
        break;
      case CLIMB_DOWN:
        power = climb();
        break;
      default:
        if (isLocked()) {
          setLocked(false);
        }
        power = calcPID();
        break;
    }

    if (isLocked() && power != 0) { // XXX: might slip too fast on climb weight
      servoAlert.set(true);
    }

    leftMotor.set(power);
    rightMotor.set(power);

  }
}
