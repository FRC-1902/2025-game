// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FloorIntake;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import static edu.wpi.first.units.Units.Inches;

/** Add your docs here. */
public class FloorSim implements FloorBase {

    IntakeSimulation intakeSim; 
    
    FloorBaseInputs inputs;
    double rollerSpeed = 0;
    double motorReduction = 1; 
    Rotation2d targetAngle;
    SingleJointedArmSim armSim; 
    DCMotor gearbox;
    PIDController pid; 
    Pose3d intakePose; 

    public FloorSim() {
        
        inputs = new FloorBaseInputs();

        gearbox = DCMotor.getNEO(1);

        armSim = new SingleJointedArmSim(gearbox, FloorConstants.SimultationConstants.GEARING,
                FloorConstants.SimultationConstants.MOMENT, FloorConstants.SimultationConstants.ARM_LENGTH,
                FloorConstants.Positions.MIN_PIVOT.getRadians(),
                FloorConstants.Positions.MAX_PIVOT.getRadians(), FloorConstants.SimultationConstants.SIMULATE_GRAVITY,
                FloorConstants.Positions.DEFAULT_ANGLE);

        pid = new PIDController(FloorConstants.PIDConstants.PIVOT_P, FloorConstants.PIDConstants.PIVOT_I,
                FloorConstants.PIDConstants.PIVOT_D);
                pid.disableContinuousInput(); // Makes sure that intake doesn't try to gas it through the floor
                pid.setTolerance(FloorConstants.Positions.TOLERANCE.getDegrees());
                pid.setIZone(10);

        // Sim setup stuff
        
         /*  if (RobotContainer.MAPLESIM) {
            intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Coral",
                null,
                Inches.of(5),
                Inches.of(5),
                IntakeSide.FRONT,
                1
            );
          }
            */
        intakePose = new Pose3d(new Translation3d(0, 0.0, 0), new Rotation3d(-180.0, 20.54, -180.0));
        targetAngle = Rotation2d.fromDegrees(FloorConstants.Positions.DEFAULT_ANGLE);
         
    }

    @Override
    public void setSpeed(double speed) {
        rollerSpeed = speed;
    };

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(armSim.getAngleRads()); 
    };

    @Override
    public void setAngle(Rotation2d angle) {
        targetAngle = angle;
    };

    @Override
    public boolean hasCoral() {
        return inputs.hasCoral;
    };

    @Override
    public void resetPID(){
        pid.reset();
    };

    public boolean atSetpoint(){
       return pid.atSetpoint();
    }

    private double pidCalc(){
        return pid.calculate(getAngle().getDegrees(), targetAngle.getDegrees());
    }
    private void updateTelemetry(){
        intakePose = new Pose3d(new Translation3d(), new Rotation3d(0.0, targetAngle.getDegrees(), 0.0)); 
        Logger.recordOutput("FloorIntake", intakePose);
    }

    @Override
    public void update(FloorBaseInputs inputs) {
        if (Robot.isReal()) return;
        // Setup Sim Logic here
        inputs.atSetpoint = atSetpoint(); 
        armSim.setInputVoltage(pidCalc());
        updateTelemetry();
        Logger.recordOutput("TargetAngle", targetAngle.getDegrees());
    };
}
