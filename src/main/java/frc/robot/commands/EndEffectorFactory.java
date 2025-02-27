// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

/** Add your docs here. */
public class EndEffectorFactory {
    private final EndEffectorSubsystem endEffector;

    public EndEffectorFactory(EndEffectorSubsystem endEffector){
        this.endEffector = endEffector;
    }

    public Command getIndexSequence() {
        return new SequentialCommandGroup(
            new EndEffectorCommand(
                endEffector, -0.1,
                () -> endEffector.isBackPieceSensorActive()
            ),
            new EndEffectorCommand(
                endEffector, 0.05,
                () -> !endEffector.isBackPieceSensorActive() && endEffector.isFrontPieceSensorActive()
            )
        );
    }
}
