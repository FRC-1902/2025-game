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
public class ElevatorDownFactory {
  private final EndEffectorSubsystem endEffectorSubsystem;
  private final ElevatorSubsystem elevatorSubsystem; 
  private final FloorIntakeSubsystem floorIntakeSubsystem;

  public ElevatorDownFactory(EndEffectorSubsystem endEffectorSubsystem, ElevatorSubsystem elevatorSubsystem, FloorIntakeSubsystem floorIntakeSubsystem){
    this.endEffectorSubsystem = endEffectorSubsystem; 
    this.elevatorSubsystem = elevatorSubsystem;
    this.floorIntakeSubsystem = floorIntakeSubsystem;
  }

  public Command getPosition(){
    return new SequentialCommandGroup(
      new ElevatorCommand(elevatorSubsystem, Constants.Elevator.Position.MIN), 
      new DeployFloorIntakeCommand(Rotation2d.fromDegrees(0), elevatorSubsystem, floorIntakeSubsystem, endEffectorSubsystem)
    );
  }
}
    