// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeIntake.AlgaeBase.AlgaeBaseInputs;

/** Add your docs here. */
public class AlgaeSim implements AlgaeBase {

 AlgaeBaseInputs inputs; 
 double rollerSpeed = 0;
 Rotation2d targetAngle;
    IntakeSimulation intakeSim = null;

    public AlgaeSim() {
        inputs = new AlgaeBaseInputs();
        //Sim setup stuff
       /*  if (RobotContainer.MAPLESIM) {
            intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Algae",
                RobotContainer.instance.drivetrain.getDriveSim(),
                Inches.of(15),
                Inches.of(6),
                IntakeSide.LEFT,
                1
            );
        }
            */
    }


    @Override
    public void setSpeed(double speed) {
        rollerSpeed = speed;
    }

    @Override
    public void setAngle(Rotation2d angle){
        targetAngle = angle;
    }

    @Override
    public void update(AlgaeBaseInputs inputs) {
        if (Robot.isReal()) return;
        //Setup Sim Logic here
    }

    @Override
    public Rotation2d getAngle() {
        if (Robot.isReal()) return Rotation2d.fromDegrees(0);
        return targetAngle;
    }

    @Override
    public boolean hasAlgae() {
        return inputs.hasAlgae;
    }

    @Override
    public void resetPID() {}
    
}
