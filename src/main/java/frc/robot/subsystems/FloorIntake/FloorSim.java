// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FloorIntake;

import org.ironmaple.simulation.IntakeSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

/** Add your docs here. */
public class FloorSim implements FloorBase {

    IntakeSimulation intakeSim = null;
    FloorBaseInputs inputs;
    double rollerSpeed = 0;
    Rotation2d targetAngle;

    public FloorSim() {
        inputs = new FloorBaseInputs();
        // Sim setup stuff
        /*
         * if (RobotContainer.MAPLESIM) {
         * intakeSim = IntakeSimulation.OverTheBumperIntake(
         * "Floor",
         * RobotContainer.instance.drivetrain.getDriveSim(),
         * Inches.of(15),
         * Inches.of(6),
         * IntakeSide.LEFT,
         * 1
         * );
         * }
         */
    }

    @Override
    public void setSpeed(double speed) {
        rollerSpeed = speed;
    };

    @Override
    public Rotation2d getAngle() {
        if (Robot.isReal())
            return Rotation2d.fromDegrees(0);
        return targetAngle;
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
    public void resetPID() {
    };

    @Override
    public void update(FloorBaseInputs inputs) {
        if (Robot.isReal())
            return;
        // Setup Sim Logic here
    };
}
