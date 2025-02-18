package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Elevator.Position;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;

public class DeployFloorIntakeCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final FloorIntakeSubsystem floorIntakeSubsystem;
  private final EndEffectorSubsystem endEffectorSubsystem;

  private Rotation2d targetAngle;

  /** Creates a new DeployFloorIntakeCommand. */
  public DeployFloorIntakeCommand(
      Rotation2d targetAngle,
      ElevatorSubsystem elevatorSubsystem, 
      FloorIntakeSubsystem floorIntakeSubsystem, 
      EndEffectorSubsystem endEffectorSubsystem
    ){
    
    this.elevatorSubsystem = elevatorSubsystem; 
    this.floorIntakeSubsystem = floorIntakeSubsystem;
    this.endEffectorSubsystem = endEffectorSubsystem;

    this.targetAngle = targetAngle;

    addRequirements(floorIntakeSubsystem);
  }

  @Override
  public void initialize() {
    if(
        endEffectorSubsystem.isFrontPieceSensorActive() ||
        floorIntakeSubsystem.pieceSensorActive() ||
        !elevatorSubsystem.isAtPosition(Position.MIN)
      ){
      DataLogManager.log("Command shouldn't start");
      return;
    }
    floorIntakeSubsystem.setAngle(targetAngle);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return floorIntakeSubsystem.atSetpoint();
  }
}
