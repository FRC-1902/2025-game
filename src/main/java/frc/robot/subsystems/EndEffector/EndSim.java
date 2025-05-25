// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import org.ironmaple.simulation.IntakeSimulation;

/** Add your docs here. */
public class EndSim implements EndBase {

IntakeSimulation intakeSim = null;
    EndBaseInputs inputs;
    double rollerSpeed = 0;

    public EndSim(){
        inputs = new EndBaseInputs();
        //Simmies go here
    }

    @Override
    public void setSpeed(double speed){
        rollerSpeed = speed;
    };

    @Override
    public boolean hasCoralFront(){
        return inputs.hasCoralFront;
    };

    @Override
    public boolean hasCoralBack(){
        return inputs.hasCoralBack;
    };

    @Override
    public void update(EndBaseInputs inputs){
        //Sim logic here 
    };
}
