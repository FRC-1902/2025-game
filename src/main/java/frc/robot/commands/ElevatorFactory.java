// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.Position;
import frc.robot.commands.intake.DeployFloorIntakeCommand;   

/** Add your docs here. */
public class ElevatorFactory {
  private final EndEffectorSubsystem endEffectorSubsystem;
  private final ElevatorSubsystem elevatorSubsystem; 
  private final FloorIntakeSubsystem floorIntakeSubsystem;
  private final EndEffectorFactory endEffectorFactory;

  public ElevatorFactory(EndEffectorSubsystem endEffectorSubsystem, ElevatorSubsystem elevatorSubsystem, FloorIntakeSubsystem floorIntakeSubsystem){
    this.endEffectorSubsystem = endEffectorSubsystem; 
    this.elevatorSubsystem = elevatorSubsystem;
    this.floorIntakeSubsystem = floorIntakeSubsystem;

    endEffectorFactory = new EndEffectorFactory(endEffectorSubsystem);
  }


  /**
   * Returns a command that sets the elevator to the target position while deploying the intake out of the way of the elevator
   * @param targetPosition
   * @return
   */
  public Command getElevatorCommand(Position targetPosition){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new DeployFloorIntakeCommand(Rotation2d.fromDegrees(Constants.FloorIntake.ELEVATOR_ANGLE), elevatorSubsystem, floorIntakeSubsystem),
        endEffectorFactory.getIndexSequence()
      ),
      new ElevatorCommand(elevatorSubsystem, targetPosition)
    );
  }

  /**
   * Returns a command that sets the elevator to MIN Position and retracts intake
   * @return
   */
  public Command getElevatorDownCommand(){
    return new SequentialCommandGroup(
      endEffectorFactory.getIndexSequence(),
      new ElevatorCommand(elevatorSubsystem, Position.MIN),
      new DeployFloorIntakeCommand(Rotation2d.fromDegrees(Constants.FloorIntake.DEFAULT_ANGLE), elevatorSubsystem, floorIntakeSubsystem)
    ).finallyDo(() -> {
      elevatorSubsystem.setPosition(Constants.Elevator.Position.MIN);
    });
  }
}
    