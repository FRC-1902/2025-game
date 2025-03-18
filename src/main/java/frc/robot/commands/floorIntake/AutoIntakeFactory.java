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
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.endEffector.EndEffectorFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoIntakeFactory {
  FloorIntakeSubsystem floorIntakeSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  EndEffectorSubsystem endEffectorSubsystem;
  EndEffectorFactory endEffectorFactory;
  LEDSubsystem led;

  /**
   * returns command composition that correctly sucks piece off the floor, retracts, and indexes for scoring.
   * @param floorIntakeSubsystem
   * @param elevatorSubsystem
   * @param endEffectorSubsystem
   * @param endEffectorFactory
   * @param led
   */
  public AutoIntakeFactory(FloorIntakeSubsystem floorIntakeSubsystem, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, LEDSubsystem led) {
    this.floorIntakeSubsystem = floorIntakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.endEffectorSubsystem = endEffectorSubsystem;
    this.endEffectorFactory = endEffectorFactory;
    this.led = led;

    endEffectorFactory = new EndEffectorFactory(endEffectorSubsystem);
  }

  /**
   * 
   * @param angle referring to deployed angle
   * @returns command composition for picking up and indexing game piece
   */
  // public Command getIntakeSequence(double angle) {
  //   // Move elevator down and intake out to specified deployed position
  //   return new SequentialCommandGroup(
  //     new ParallelCommandGroup(
  //       new ElevatorCommand(
  //         elevatorSubsystem, 
  //         Constants.Elevator.Position.MIN
  //       ),
  //       new PositionIntakeCommand(
  //         Rotation2d.fromDegrees(angle),
  //         elevatorSubsystem, 
  //         floorIntakeSubsystem 
  //       )
  //     ),
  //     // Intake game piece
  //     new IntakeCommand(floorIntakeSubsystem, led)
  //   ).finallyDo((wasCancelled) -> {
  //     new ConditionalCommand(
  //       // Move floor intake in and index successful intake
  //       new SequentialCommandGroup(
  //         new PositionIntakeCommand(
  //           Rotation2d.fromDegrees(Constants.FloorIntake.DEFAULT_ANGLE), 
  //           elevatorSubsystem,
  //           floorIntakeSubsystem
  //         ),
  //         new IndexCommand( 
  //           floorIntakeSubsystem, 
  //           endEffectorSubsystem
  //         ),
  //         // Make sure piece is aligned with index sequence
  //         endEffectorFactory.getIndexSequence()
  //       ),
  //       // clean up failed intake
  //       new SequentialCommandGroup(
  //         // Runs outtake for .5 seconds before cancelling
  //         new ParallelDeadlineGroup(
  //           new WaitCommand(.5), 
  //           new OuttakeCommand(floorIntakeSubsystem)
  //         ),          
  //         // Move floor intake back in after running outtake
  //         new PositionIntakeCommand(
  //           Rotation2d.fromDegrees(Constants.FloorIntake.DEFAULT_ANGLE), 
  //           elevatorSubsystem,
  //           floorIntakeSubsystem
  //         )
  //       ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming), // Gives interuption behavior where it cancels incoming command  
  //       () -> floorIntakeSubsystem.pieceSensorActive()
  //     ).schedule();
  //   });
  // }

  /**
   * Auto intake sequence for autonomous with cleanup handled. Doesn't have finallyDo behavior after end.
   * @param angle referring to deploying angle
   * @returns intake sequence specifically for auto. 
   */
  public Command getAutonomousIntakeSequence(double angle) {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ElevatorCommand(
          elevatorSubsystem, 
          Constants.Elevator.Position.MIN
        ),
        new PositionIntakeCommand(
          Rotation2d.fromDegrees(angle),
          floorIntakeSubsystem 
        )
      ),
      new IntakeCommand(floorIntakeSubsystem, led)
    );
  }

  public Command getAutonomousIndexSequence() {
    return new SequentialCommandGroup(
      new PositionIntakeCommand(
        Rotation2d.fromDegrees(Constants.FloorIntake.DEFAULT_ANGLE), 
        floorIntakeSubsystem
      ),
      new IndexCommand( 
        floorIntakeSubsystem, 
        endEffectorSubsystem
      ),
      endEffectorFactory.getIndexSequence()
    );
  }


  public Command getIntakeSequence(double angle) {
    // Move elevator down and intake out to specified deployed position
    return new SequentialCommandGroup(
      new InstantCommand(() -> elevatorSubsystem.setPosition(Constants.Elevator.Position.MIN)),
      new PositionIntakeCommand(Rotation2d.fromDegrees(angle),floorIntakeSubsystem),
      // Intake game piece
      new InstantCommand(()-> DataLogManager.log("Intake Command Started")),
      new IntakeCommand(floorIntakeSubsystem, led),
      new InstantCommand(()-> DataLogManager.log("Intake Command Ended"))
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).finallyDo((wasCancelled) -> {
      new ConditionalCommand(
        // Move floor intake in and index successful intake
        new SequentialCommandGroup(
          new InstantCommand(()-> DataLogManager.log("Good finish Started")),
          new ParallelDeadlineGroup(
            new PositionIntakeCommand(
              Rotation2d.fromDegrees(Constants.FloorIntake.ELEVATOR_ANGLE), 
              floorIntakeSubsystem
            ),
            new ElevatorCommand(elevatorSubsystem, Constants.Elevator.Position.MIN)
          ),
          new PositionIntakeCommand(Rotation2d.fromDegrees(Constants.FloorIntake.DEFAULT_ANGLE), floorIntakeSubsystem),
          new IndexCommand(floorIntakeSubsystem, endEffectorSubsystem), 
          endEffectorFactory.getIndexSequence()
        ),
        // clean up failed intake
        new SequentialCommandGroup(
          new InstantCommand(()-> DataLogManager.log("Bad finish Started")),
          new ParallelDeadlineGroup(
            new ParallelDeadlineGroup(
              new ElevatorCommand(elevatorSubsystem, Constants.Elevator.Position.MIN), 
              new PositionIntakeCommand(Rotation2d.fromDegrees(Constants.FloorIntake.ELEVATOR_ANGLE), floorIntakeSubsystem)
            ), 
            new OuttakeCommand(floorIntakeSubsystem)
          ),     
          // Move floor intake back in after running outtake
          new PositionIntakeCommand(
            Rotation2d.fromDegrees(Constants.FloorIntake.DEFAULT_ANGLE), 
            floorIntakeSubsystem
          )
        ), 
        () -> floorIntakeSubsystem.pieceSensorActive()
      ).schedule();
    });
  }
}


