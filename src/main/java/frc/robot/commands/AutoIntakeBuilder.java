// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.Constants;
import frc.robot.commands.DeployFloorIntakeCommand;
import frc.robot.commands.IntakeFloorIntakeCommand;
import frc.robot.commands.IndexFloorIntakeCommand;
import frc.robot.commands.OuttakeFloorIntakeCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

/** Add your docs here. */
public class AutoIntakeBuilder {
    FloorIntake floorIntakeSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    EndEffectorSubsystem endEffectorSubsystem;

    Command intakeSequence;
    Command cancelSequence;

    public Command getIntakeSequence() {
        // TODO: set rotation angles
        intakeSequence = new SequentialCommandGroup(
                new ConditionalCommand(
                        new ElevatorCommand(floorIntakeSubsystem, elevatorSubsystem, Constants.Elevator.Position.MIN),

                        new SequentialCommandGroup(
                                new DeployFloorIntakeCommand(new Rotation2d(), elevatorSubsystem, floorIntakeSubsystem,
                                        endEffectorSubsystem),
                                new ElevatorCommand(floorIntakeSubsystem, elevatorSubsystem,
                                        Constants.Elevator.Position.MIN)),
                        () -> (elevatorSubsystem.getPosition() != Constants.Elevator.Position.MIN.getHeight() &&
                                floorIntakeSubsystem.getAngle().getDegrees() >= 45)),

                new DeployFloorIntakeCommand(new Rotation2d(), elevatorSubsystem, floorIntakeSubsystem,
                        endEffectorSubsystem),
                new IntakeFloorIntakeCommand(floorIntakeSubsystem)).finallyDo((wasCancelled) -> {
                    new ConditionalCommand(
                            new SequentialCommandGroup(
                                    new DeployFloorIntakeCommand(
                                            new Rotation2d(),
                                            elevatorSubsystem,
                                            floorIntakeSubsystem,
                                            endEffectorSubsystem),
                                    new IndexFloorIntakeCommand(floorIntakeSubsystem, endEffectorSubsystem,
                                            elevatorSubsystem)),
                            new SequentialCommandGroup(
                                    new DeployFloorIntakeCommand(new Rotation2d(), elevatorSubsystem,
                                            floorIntakeSubsystem, endEffectorSubsystem),
                                    new OuttakeFloorIntakeCommand(floorIntakeSubsystem),
                                    new DeployFloorIntakeCommand(new Rotation2d(), elevatorSubsystem,
                                            floorIntakeSubsystem, endEffectorSubsystem)),
                            () -> floorIntakeSubsystem.pieceSensorActive()).schedule();
                })

        ;
        return intakeSequence;
    }

}
