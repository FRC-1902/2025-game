package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Elevator.Position;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;

public class DeployFloorIntakeCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final FloorIntakeSubsystem floorIntakeSubsystem;

  private Rotation2d targetAngle;

  /** Creates a new DeployFloorIntakeCommand. */
  public DeployFloorIntakeCommand(
      Rotation2d targetAngle,
      ElevatorSubsystem elevatorSubsystem, 
      FloorIntakeSubsystem floorIntakeSubsystem
    ){
    
    this.elevatorSubsystem = elevatorSubsystem; 
    this.floorIntakeSubsystem = floorIntakeSubsystem;
    this.targetAngle = targetAngle;

    addRequirements(floorIntakeSubsystem);
  }

  @Override
  public void initialize() {
    if(!elevatorSubsystem.isAtPosition(Position.MIN)){
      DataLogManager.log("Command shouldn't start");
      return;
    }
    floorIntakeSubsystem.setAngle(targetAngle);
  }
  
  /**
   * Actively sucks coral in while moving to make sure it doesn't fly out
   */
  @Override
  public void execute() {
    if (floorIntakeSubsystem.pieceSensorActive()) {
      floorIntakeSubsystem.setSpeed(1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    floorIntakeSubsystem.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return floorIntakeSubsystem.atSetpoint();
  }
}
