// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;



/** Add your docs here. */
public class NonLinearProfile {
    private double input;

    public NonLinearProfile(double input){
        this.input = input;
    }

    // functions for each scenario
    // y(x=1) = 0
    // y(x>1)=0.08+(x-.03)^3
    // y(x<1)=-0.11+(x+.04)^3
    public double scaleInputs(){
        double output = 0;

        if(input > 0){
             output = 0.08 + Math.pow((input - 0.03), 3); 
        }
        else if(input < 0){
             output = -0.11 + Math.pow((input + 0.04), 3);
        }
        else{
             output = 0;
        }
        return output;
    }
}
