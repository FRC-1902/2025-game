package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.Constants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.EndEffectorFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoIntakeFactory {
	FloorIntakeSubsystem floorIntakeSubsystem;
	ElevatorSubsystem elevatorSubsystem;
	EndEffectorSubsystem endEffectorSubsystem;
	EndEffectorFactory endEffectorFactory;

	public AutoIntakeFactory(FloorIntakeSubsystem floorIntakeSubsystem, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, EndEffectorFactory endEffectorFactory) {
		this.floorIntakeSubsystem = floorIntakeSubsystem;
		this.elevatorSubsystem = elevatorSubsystem;
		this.endEffectorSubsystem = endEffectorSubsystem;
		this.endEffectorFactory = endEffectorFactory;
	}

	public Command getIntakeSequence(double angle) {
		// TODO: set rotation angles
		return new SequentialCommandGroup(
			new ParallelCommandGroup(
				new ElevatorCommand(
					elevatorSubsystem, 
					Constants.Elevator.Position.MIN
				),
				new DeployFloorIntakeCommand(
					Rotation2d.fromDegrees(angle), // todo: double check zero reference -> out deploy
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
					),
					new InstantCommand(
						() -> endEffectorFactory.getIndexSequence(), 
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
				).withInterruptBehavior(InterruptionBehavior.kCancelIncoming), // XXX: verify behavior
				() -> floorIntakeSubsystem.pieceSensorActive()
			).schedule();
		});
	}
}
