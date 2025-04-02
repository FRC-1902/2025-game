// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.EndEffectorSubsystem;

/**
 * Returns command composition(s) to re-align game piece. Makes sure that game piece position in the end effector is consistent.
 */
public class EndEffectorFactory {
  private final EndEffectorSubsystem endEffector;

  public EndEffectorFactory(EndEffectorSubsystem endEffector){
    this.endEffector = endEffector;
  }

  /**
   * 
   * @returns command to correctly index game piece after being fed by Floor Intake.
   */
  public Command getIndexSequence() {
    return new ConditionalCommand(
      new SequentialCommandGroup(
        new EndEffectorCommand(
          endEffector, 
          -0.1,
          () -> endEffector.isBackPieceSensorActive()
        ),
        new EndEffectorCommand(
          endEffector, 
          0.1,
          () -> !endEffector.isBackPieceSensorActive() && endEffector.isFrontPieceSensorActive()
        )
      ),
      new InstantCommand(),
      () -> {return endEffector.isBackPieceSensorActive() || endEffector.isFrontPieceSensorActive();}
    );
  }

  /**
   * 
   * @returns command to Re-Index game piece after initial indexing sequence.
   */
  public Command getReverseSequence() {
    return new SequentialCommandGroup(
      new EndEffectorCommand(
        endEffector, 
        -0.1,
        () -> endEffector.isBackPieceSensorActive()
      ),
      new EndEffectorCommand(
        endEffector, 
        0.1,
        () -> !endEffector.isBackPieceSensorActive() && endEffector.isFrontPieceSensorActive()
      )
    );
  }
}
