package frc.robot.commands.floorIntake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.LED;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.endEffector.EndEffectorFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class AutoIntakeFactory {
  FloorIntakeSubsystem floorIntakeSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  EndEffectorSubsystem endEffectorSubsystem;
  EndEffectorFactory endEffectorFactory;
  LEDSubsystem led;

  public AutoIntakeFactory(FloorIntakeSubsystem floorIntakeSubsystem, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, EndEffectorFactory endEffectorFactory, LEDSubsystem led) {
    this.floorIntakeSubsystem = floorIntakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.endEffectorSubsystem = endEffectorSubsystem;
    this.endEffectorFactory = endEffectorFactory;
    this.led = led;
  }

  public Command getIntakeSequence(double angle) {
    // TODO: set rotation angles
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ElevatorCommand(
          elevatorSubsystem, 
          Constants.Elevator.Position.MIN
        ),
        new PositionIntakeCommand(
          Rotation2d.fromDegrees(angle),
          elevatorSubsystem, 
          floorIntakeSubsystem 
        )
      ),
      new IntakeCommand(floorIntakeSubsystem, led)
    ).finallyDo((wasCancelled) -> {
      new ConditionalCommand(
        // index successful intake
        new SequentialCommandGroup(
          new PositionIntakeCommand(
            Rotation2d.fromDegrees(Constants.FloorIntake.DEFAULT_ANGLE), // todo: double check -> bring it in
            elevatorSubsystem,
            floorIntakeSubsystem
          ),
          new IndexCommand( 
            floorIntakeSubsystem, 
            endEffectorSubsystem
          ),
          endEffectorFactory.getIndexSequence()
        ),
        // clean up failed intake
        new SequentialCommandGroup(
          // XXX: may not want this initial move out for cleanup
          // new DeployFloorIntakeCommand(
          //   Rotation2d.fromDegrees(110), // todo: check # could be horrible
          //   elevatorSubsystem,
          //   floorIntakeSubsystem, 
          //   endEffectorSubsystem
          // ),
          new ParallelDeadlineGroup(
            new WaitCommand(.5), 
            new OuttakeCommand(floorIntakeSubsystem)
          ),          
          // new OuttakeCommand(floorIntakeSubsystem),
          new PositionIntakeCommand(
            Rotation2d.fromDegrees(Constants.FloorIntake.DEFAULT_ANGLE), // todo: check #
            elevatorSubsystem,
            floorIntakeSubsystem
          )
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming), 
        () -> floorIntakeSubsystem.pieceSensorActive()
      ).schedule();
    });
  }
  public Command getAutonomousIntakeSequence(double angle) {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ElevatorCommand(
          elevatorSubsystem, 
          Constants.Elevator.Position.MIN
        ),
        new PositionIntakeCommand(
          Rotation2d.fromDegrees(angle),
          elevatorSubsystem, 
          floorIntakeSubsystem 
        )
      ),
      new IntakeCommand(floorIntakeSubsystem, led),

      new ConditionalCommand(
        // index successful intake
        new SequentialCommandGroup(
          new PositionIntakeCommand(
            Rotation2d.fromDegrees(Constants.FloorIntake.DEFAULT_ANGLE), // todo: double check -> bring it in
            elevatorSubsystem,
            floorIntakeSubsystem
          ),
          new IndexCommand( 
            floorIntakeSubsystem, 
            endEffectorSubsystem
          ),
          endEffectorFactory.getIndexSequence()
        ),

        // clean up failed intake
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new WaitCommand(.5), 
            new OuttakeCommand(floorIntakeSubsystem)
          ),          
          // new OuttakeCommand(floorIntakeSubsystem),
          new PositionIntakeCommand(
            Rotation2d.fromDegrees(Constants.FloorIntake.DEFAULT_ANGLE), // todo: check #
            elevatorSubsystem,
            floorIntakeSubsystem
          )
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming), 
        
        () -> floorIntakeSubsystem.pieceSensorActive()
      )
    );
  }
}
