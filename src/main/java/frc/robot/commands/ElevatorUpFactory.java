// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.Position;
import frc.robot.commands.intake.DeployFloorIntakeCommand;   

/** Add your docs here. */
public class ElevatorUpFactory {
  private final EndEffectorSubsystem endEffectorSubsystem;
  private final ElevatorSubsystem elevatorSubsystem; 
  private final FloorIntakeSubsystem floorIntakeSubsystem;

  public ElevatorUpFactory(EndEffectorSubsystem endEffectorSubsystem, ElevatorSubsystem elevatorSubsystem, FloorIntakeSubsystem floorIntakeSubsystem){
    this.endEffectorSubsystem = endEffectorSubsystem; 
    this.elevatorSubsystem = elevatorSubsystem;
    this.floorIntakeSubsystem = floorIntakeSubsystem;
  }

  public Command getPosition(Position targetPosition){
    return new SequentialCommandGroup(
      new DeployFloorIntakeCommand(Rotation2d.fromDegrees(61), elevatorSubsystem, floorIntakeSubsystem, endEffectorSubsystem), // todo: figure out what resting angle should be at
      new ElevatorCommand(elevatorSubsystem, targetPosition)
    );
  }
}
    