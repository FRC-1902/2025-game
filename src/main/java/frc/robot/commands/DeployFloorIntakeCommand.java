// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeployFloorIntakeCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final FloorIntake floorIntakeSubsystem;
  private final EndEffectorSubsystem endEffectorSubsystem;
  /** Creates a new DeployFloorIntakeCommand. */
  public DeployFloorIntakeCommand(ElevatorSubsystem elevatorSubsystem, FloorIntake floorIntakeSubsystem, EndEffectorSubsystem endEffectorSubsystem){
    this.elevatorSubsystem = elevatorSubsystem; 
    this.floorIntakeSubsystem = floorIntakeSubsystem;
    this.endEffectorSubsystem = endEffectorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(floorIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!endEffectorSubsystem.isFrontPieceSensorActive() && !floorIntakeSubsystem.pieceSensorActive()){
      if(elevatorSubsystem.getPosition() == Constants.Elevator.Position.MIN.getHeight()){
        floorIntakeSubsystem.setAngle(Rotation2d.fromDegrees(90)); // todo: get target down position
      }
    }
    else{
      floorIntakeSubsystem.setAngle(Rotation2d.fromDegrees(0)); // todo: get target up position
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(elevatorSubsystem.getPosition() == Constants.Elevator.Position.MIN.getHeight()){
      floorIntakeSubsystem.setAngle(Rotation2d.fromDegrees(0));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return floorIntakeSubsystem.pieceSensorActive() || endEffectorSubsystem.isFrontPieceSensorActive();
  }
}
