// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;
import frc.robot.subsystems.FloorIntake.FloorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstants.Position;
import frc.robot.subsystems.FloorIntake.FloorConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class ElevatorFactory {
    
    FloorSubsystem floorSubsystem; 
    ElevatorSubsystem elevatorSubsystem; 

    public ElevatorFactory(FloorSubsystem floorSubsystem, ElevatorSubsystem elevatorSubsystem){
        this.floorSubsystem = floorSubsystem; 
        this.elevatorSubsystem = elevatorSubsystem; 
    }

    public Command elevatorPositioning(Position target){
        return elevatorSubsystem.setPosition(target).alongWith(floorSubsystem.setPivotAngle(Rotation2d.fromDegrees(FloorConstants.Positions.ELEVATOR_ANGLE))); 
    }
}
