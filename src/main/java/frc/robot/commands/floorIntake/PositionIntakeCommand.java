package frc.robot.commands.floorIntake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.Position;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;

public class PositionIntakeCommand extends Command {
  private final FloorIntakeSubsystem floorIntakeSubsystem;

  private Rotation2d targetAngle;

  /**
   * Sets floor intake to target angle. If piece detected while moving, sucks in on piece.
   * @param targetAngle
   * @param floorIntakeSubsystem
   */
  public PositionIntakeCommand(Rotation2d targetAngle, FloorIntakeSubsystem floorIntakeSubsystem){
    this.floorIntakeSubsystem = floorIntakeSubsystem;
    this.targetAngle = targetAngle;

    addRequirements(floorIntakeSubsystem);
  }

  /**
   * Checks to make sure that elevator is at the MIN (see constants) position before setting new target angle.
   */
  @Override
  public void initialize() {
    floorIntakeSubsystem.setAngle(targetAngle);
  }
  
  /**
   * Actively sucks coral in while moving to make sure it doesn't fly out
   */
  @Override
  public void execute() {
    if (floorIntakeSubsystem.pieceSensorActiveFiltered()) {
      floorIntakeSubsystem.setSpeed(1); // Runs intake to keep coral in the intake while rotating intake.
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (floorIntakeSubsystem.pieceSensorActiveFiltered()) {
      floorIntakeSubsystem.setSpeed(0);
    }
  }

  @Override
  public boolean isFinished() {
    return floorIntakeSubsystem.atSetpoint();
  }
}
