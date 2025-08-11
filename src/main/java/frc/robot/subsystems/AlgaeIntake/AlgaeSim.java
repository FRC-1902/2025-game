// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.function.DoubleSupplier;


/** Add your docs here. */
public class AlgaeSim implements AlgaeBase {

    DoubleSupplier elevatorHeight;  
    double rollerSpeed = 0;
    Rotation2d targetAngle;
    SingleJointedArmSim algaeSim; 
    PIDController pid; 

    public AlgaeSim(DoubleSupplier elevatorHeight) {

        this.elevatorHeight = elevatorHeight; 

        algaeSim = new SingleJointedArmSim(
            DCMotor.getNEO(1), 
            AlgaeConstants.Simulation.GEARING, 
            AlgaeConstants.Simulation.MOMENT,
            AlgaeConstants.Simulation.ARM_LENGTH, 
            AlgaeConstants.Positions.MIN_PIVOT.getRadians(), 
            AlgaeConstants.Positions.MAX_PIVOT.getRadians(), 
            AlgaeConstants.Simulation.SIMULATE_GRAVITY, 
            AlgaeConstants.Positions.DEFAULT_ANGLE.getRadians()
            ); 
        
        pid = new PIDController(AlgaeConstants.PID.kP, AlgaeConstants.PID.kI, AlgaeConstants.PID.kD); 
        targetAngle = AlgaeConstants.Positions.DEFAULT_ANGLE;

    }

    @Override
    public void setSpeed(double speed) {
        rollerSpeed = speed;
    }

    @Override
    public void setAngle(Rotation2d angle) {
        targetAngle = angle;
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(algaeSim.getAngleRads());
    }

    @Override
    public boolean hasAlgae() {
        return false; // TODO: implement with maplesim
    }

    public void resetPID(){
        pid.reset();
    }

    private double pidCalc(){
        return pid.calculate(getAngle().getDegrees(), targetAngle.getDegrees()); 
    }

    private void updateTelemetry(){ 
        Pose3d algaePose = new Pose3d(new Translation3d(0.3, 0.0, 0.425 + elevatorHeight.getAsDouble()), new Rotation3d(0.0, getAngle().getRadians() * -1, 0.0));
        Logger.recordOutput("AlgaeIntake/algaePose", algaePose);
    }

    @Override
    public void update(AlgaeBaseInputs inputs) {
        double power = pidCalc(); 

        inputs.atSetpoint = pid.atSetpoint(); 

       if(DriverStation.isEnabled()) 
        algaeSim.setInputVoltage(power * 12);

        algaeSim.update(0.02); 
        updateTelemetry();
    }
}