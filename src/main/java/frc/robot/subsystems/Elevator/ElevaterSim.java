// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator.ElevatorConstants.Position;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class ElevaterSim implements ElevatorBase {

  Position targetPosition;
  boolean locked;
  ElevatorSim elevatorSim;
  PIDController pid;
  double unlockTime, climbLockTime;

  public ElevaterSim() {

    elevatorSim = new ElevatorSim(DCMotor.getNEO(2), ElevatorConstants.SimulationConstants.SimSetup.GEARING,
        ElevatorConstants.SimulationConstants.SimSetup.CARRIAGE_MASS,
        ElevatorConstants.SimulationConstants.SimSetup.DRUM_RADIUS,
        ElevatorConstants.SimulationConstants.SimSetup.MIN_HEIGHT,
        ElevatorConstants.SimulationConstants.SimSetup.MAX_HEIGHT,
        ElevatorConstants.SimulationConstants.SimSetup.SIMULATE_GRAVITY,
        ElevatorConstants.SimulationConstants.SimSetup.STARTING_HEIGHT);

    pid = new PIDController(ElevatorConstants.SimulationConstants.PID.kP,
        ElevatorConstants.SimulationConstants.PID.kI, ElevatorConstants.SimulationConstants.PID.kD);

    targetPosition = ElevatorConstants.Position.HOME;
    resetPID();
  }

  public double getPosition() {
    return elevatorSim.getPositionMeters();
  };

  public void setPosition(Position position) {
    targetPosition = position;
    pid.setSetpoint(targetPosition.getHeight());
  };

  public boolean limitSwitchTriggered() {
    return elevatorSim.wouldHitLowerLimit(getPosition()); // TODO: Some number for the lower limit of the elevator
  };

  public boolean atSetpoint() {
    return 0.01 >= Math.abs(getPosition() - targetPosition.getHeight());
  };

  public void resetPID() {
    pid.reset();
  };

  public boolean isLocked() {
    return locked;
  };

  public void setLocked(boolean lock) {
    unlockTime = Timer.getFPGATimestamp();
    locked = lock;
  };

  /**
   * go to the bottom of the elevator to re-home
   */
  private double home() {
    if (!limitSwitchTriggered()) {
      return -0.2; 
    } else {
      return 0;
    }
  }

  private double climb() {
    if (!limitSwitchTriggered()) {
      climbLockTime = Timer.getFPGATimestamp(); 
      return -0.5; // Move down at half speed
    } else {
      // When limit switch is triggered, lock the elevator
      if (!isLocked()) {
        setLocked(true);
      }

      // Continue applying slight downward pressure for a short time after locking
      if (Timer.getFPGATimestamp() - climbLockTime < 0.2) {
        return -0.35; // Gentle downward pressure
      } else {
        return 0; // Stop motor after the settling time
      }
    }
  }

  private double calcPID() {
    if (!isLocked() && Timer.getFPGATimestamp() - unlockTime > 0.3) {
      return pid.calculate(getPosition()) + ElevatorConstants.PID.kF
          + ElevatorConstants.PID.kS * Math.signum(pid.getSetpoint() - getPosition());
    } else {
      return 0;
    }
  }

  private void updateTelemetry() {
    double currentPos = elevatorSim.getPositionMeters();
    Pose3d stageOne = new Pose3d(new Translation3d(0, 0, currentPos * 0.5), new Rotation3d()); // its kinda just as
                                                                                               // shrimple as that
    Pose3d stageTwo = new Pose3d(new Translation3d(0, 0, currentPos), new Rotation3d());

    // Logger.recordOutput("Elevator", stageOne);
    // Logger.recordOutput("EndEffector", stageTwo);
    Logger.recordOutput("Elevator/ElevatorComposition", new Pose3d[] { stageOne, stageTwo });

    Logger.recordOutput("Elevator/ElevatorCurrentPos", elevatorSim.getPositionMeters());
    Logger.recordOutput("Elevator/TargetElevatorPosition", targetPosition);
    Logger.recordOutput("Elevator/PIDPower", calcPID());
    Logger.recordOutput("Elevator/GetDraw", elevatorSim.getCurrentDrawAmps());
    Logger.recordOutput("Elevator/atSetpoint", atSetpoint());
  }

  public void update(ElevatorBaseInputs inputs) {
    if (Robot.isReal())
      return;

    
    double power;
    // Sim Logic here
    inputs.atSetpoint = atSetpoint();
    inputs.limitSwitchTriggered = limitSwitchTriggered();
    inputs.currentPosition = getPosition();
    inputs.targetPosition = targetPosition;
    inputs.isLocked = locked;

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
    
    if(DriverStation.isEnabled()){
      elevatorSim.setInputVoltage(power * 12);
    }
    else{
      elevatorSim.setInputVoltage(0);
    }

    elevatorSim.update(0.02);
    updateTelemetry();
  };

}