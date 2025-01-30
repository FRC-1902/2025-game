// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

/** Add your docs here. */
public class AutoIntakeFactory {
	FloorIntakeSubsystem floorIntakeSubsystem;
	ElevatorSubsystem elevatorSubsystem;
	EndEffectorSubsystem endEffectorSubsystem;

	public AutoIntakeFactory(FloorIntakeSubsystem floorIntakeSubsystem, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
		this.floorIntakeSubsystem = floorIntakeSubsystem;
		this.elevatorSubsystem = elevatorSubsystem;
		this.endEffectorSubsystem = endEffectorSubsystem;
	}

	public Command getIntakeSequence() {
		// TODO: set rotation angles
		return new SequentialCommandGroup(
			new ParallelCommandGroup(
				new ElevatorCommand(
					elevatorSubsystem, 
					Constants.Elevator.Position.MIN
				),
				new DeployFloorIntakeCommand(
					Rotation2d.fromDegrees(180), // todo: double check zero reference -> out deploy
					elevatorSubsystem, 
					floorIntakeSubsystem, 
					endEffectorSubsystem
				)
			),
			new IntakeFloorIntakeCommand(floorIntakeSubsystem)
		).finallyDo((wasCancelled) -> {
			new ConditionalCommand(
				// index successful intake
				new SequentialCommandGroup(
					new DeployFloorIntakeCommand(
						Rotation2d.fromDegrees(0), // todo: double check -> bring it in
						elevatorSubsystem,
						floorIntakeSubsystem,
						endEffectorSubsystem
					),
					new IndexFloorIntakeCommand( 
						floorIntakeSubsystem, 
						endEffectorSubsystem
					)
				),
				// clean up failed intake
				new SequentialCommandGroup(
					// XXX: may not want this initial move out for cleanup
					new DeployFloorIntakeCommand(
						Rotation2d.fromDegrees(110), // todo: check # could be horrible
						elevatorSubsystem,
									floorIntakeSubsystem, 
						endEffectorSubsystem
					),
					new OuttakeFloorIntakeCommand(floorIntakeSubsystem),
					new DeployFloorIntakeCommand(
						Rotation2d.fromDegrees(0), // todo: check #
						elevatorSubsystem,
						floorIntakeSubsystem, 
						endEffectorSubsystem
					)
				),
				() -> floorIntakeSubsystem.pieceSensorActive()
			).schedule();
		});
	}
}
