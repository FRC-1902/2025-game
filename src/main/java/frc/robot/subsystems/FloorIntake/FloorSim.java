// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FloorIntake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public class FloorSim implements FloorBase { 
    
    FloorBaseInputs inputs;
    double rollerSpeed;
    Rotation2d targetAngle;
    SingleJointedArmSim armSim; 
    PIDController pid; 
    Pose3d intakePose; 

    public FloorSim() {
        
        inputs = new FloorBaseInputs();

        armSim = new SingleJointedArmSim(
            DCMotor.getNEO(1), 
            FloorConstants.Simultation.GEARING,
            FloorConstants.Simultation.MOMENT, 
            FloorConstants.Simultation.ARM_LENGTH,
            FloorConstants.Positions.MIN_PIVOT.getRadians(),
            FloorConstants.Positions.MAX_PIVOT.getRadians(), 
            FloorConstants.Simultation.SIMULATE_GRAVITY,
            FloorConstants.Positions.DEFAULT_ANGLE.getRadians()
        );

        pid = new PIDController(
            FloorConstants.PID.PIVOT_P, 
            FloorConstants.PID.PIVOT_I,
            FloorConstants.PID.PIVOT_D
        );

        targetAngle = FloorConstants.Positions.DEFAULT_ANGLE;      
    }

    public void setSpeed(double speed) {
        rollerSpeed = speed;
    };

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(armSim.getAngleRads()); 
    };

    public void setAngle(Rotation2d angle) {
        targetAngle = angle;
    };

    public boolean hasCoral() {
        return inputs.hasCoral;
    };

    public void resetPID(){
        pid.reset();
    };

    public boolean atSetpoint(){
       return pid.atSetpoint();
    }

    private double pidCalc(){
        return pid.calculate(getAngle().getDegrees(), targetAngle.getDegrees())
            + FloorConstants.PID.PIVOT_G * Math.cos(getAngle().getRadians()
        );
    }

    private void updateTelemetry(){
        intakePose = new Pose3d(new Translation3d(-0.25, 0.0, 0.15), new Rotation3d(0.0, getAngle().getRadians() * -1.0, 0.0)); 
        Logger.recordOutput("FloorIntake/IntakePose", intakePose);
    }

    public void update(FloorBaseInputs inputs) {
        
        // Setup Sim Logic here
        double power = pidCalc(); 

        inputs.atSetpoint = atSetpoint(); 
        inputs.currentAngle = getAngle(); 
        inputs.targetAngle = targetAngle; 

        if (DriverStation.isEnabled())
        armSim.setInputVoltage(power * 12);

        armSim.update(0.02);
        updateTelemetry();

        Logger.recordOutput("FloorIntake/PID", power); 
    };
}