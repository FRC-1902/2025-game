// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.intake.DeployFloorIntakeCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;

/** Add your docs here. */
public class ClimbFactory {
  private final ElevatorSubsystem elevator;
  private final FloorIntakeSubsystem floorIntake;

  public ClimbFactory(ElevatorSubsystem elevator, FloorIntakeSubsystem floorIntake){
    this.elevator = elevator;
    this.floorIntake = floorIntake;
  }

  public SequentialCommandGroup getClimberUpSequence(){
    return new SequentialCommandGroup(
      new DeployFloorIntakeCommand(Rotation2d.fromDegrees(Constants.FloorIntake.FLOOR_ANGLE), elevator, floorIntake),
      new ElevatorCommand(elevator, Constants.Elevator.Position.CLIMB_UP)
    );
  }
}
