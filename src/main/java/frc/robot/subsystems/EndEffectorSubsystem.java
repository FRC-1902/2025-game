package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffectorSubsystem extends SubsystemBase {

  private SparkMax rollerMotor; 
  private DigitalInput frontSensor;
  private DigitalInput backSensor;

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem() {
    rollerMotor = new SparkMax(Constants.EndEffector.ROLLER_MOTOR_ID, MotorType.kBrushless);

    frontSensor = new DigitalInput(Constants.EndEffector.FRONT_IR_SENSOR_ID);
    backSensor = new DigitalInput(Constants.EndEffector.BACK_IR_SENSOR_ID);

    configureMotors();
  }

  private void configureMotors(){
    SparkBaseConfig rollerConfig = new SparkMaxConfig();
    // Roller Configs
    rollerConfig.idleMode(IdleMode.kCoast);
    rollerConfig.inverted(false);
    rollerConfig.disableFollowerMode(); 
    // rollerConfig.secondaryCurrentLimit(30);
    rollerConfig.smartCurrentLimit(30);
    rollerConfig.voltageCompensation(12.00);
    // ResetSafeParameters subject to change; not well documented 
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  
  /**
   * 
   * @returns if front IRsensor is activated 
   */
  public boolean isFrontPieceSensorActive(){
    return !frontSensor.get(); 
  }

  /**
   * 
   * @returns if back IRsensor is activated 
   */
  public boolean isBackPieceSensorActive(){
    return !backSensor.get(); 
  }

  /**
   * 
   * @param targetSpeed
   */
  public void setSpeed(double targetSpeed){
    rollerMotor.set(targetSpeed); // TODO: Re-Enable
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("EndEffector/Front Piece Sensor", isFrontPieceSensorActive());
    SmartDashboard.putBoolean("EndEffector/Back Piece Sensor", isBackPieceSensorActive());
  }
}
