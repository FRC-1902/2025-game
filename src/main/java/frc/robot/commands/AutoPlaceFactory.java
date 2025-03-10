package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.Position;
import frc.robot.commands.floorIntake.PositionIntakeCommand;   

/** Add your docs here. */
public class AutoPlaceFactory {
  private final EndEffectorSubsystem endEffectorSubsystem;
  private final ElevatorSubsystem elevatorSubsystem; 
  private final FloorIntakeSubsystem floorIntakeSubsystem;

  public AutoPlaceFactory(EndEffectorSubsystem endEffectorSubsystem, ElevatorSubsystem elevatorSubsystem, FloorIntakeSubsystem floorIntakeSubsystem){
    this.endEffectorSubsystem = endEffectorSubsystem; 
    this.elevatorSubsystem = elevatorSubsystem;
    this.floorIntakeSubsystem = floorIntakeSubsystem;
  }

  public Command getAutoPlace(Position targetPosition){
    return new SequentialCommandGroup(
      new PositionIntakeCommand(Rotation2d.fromDegrees(Constants.FloorIntake.ELEVATOR_ANGLE), elevatorSubsystem, floorIntakeSubsystem), // todo: figure out what resting angle should be at
      new ElevatorCommand(elevatorSubsystem, targetPosition),
      new PlaceCommand(endEffectorSubsystem),
      new ElevatorCommand(elevatorSubsystem, Constants.Elevator.Position.MIN), 
      new PositionIntakeCommand(Rotation2d.fromDegrees(Constants.FloorIntake.ELEVATOR_ANGLE), elevatorSubsystem, floorIntakeSubsystem)
    );
  }
}
    