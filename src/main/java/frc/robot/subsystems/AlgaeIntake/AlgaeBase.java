// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;


/** Add your docs here. */
@Logged
public interface AlgaeBase {

    @Logged
    public class AlgaeBaseInputs{
        // Constructor code here
        boolean hasAlgae; 
        boolean atSetpoint; 
    }

    public void setSpeed(double speed);

    public Rotation2d getAngle(); 

    public void setAngle(Rotation2d position); 

    public boolean hasAlgae();

    public void resetPID(); 

    public void update(AlgaeBaseInputs inputs);
}
