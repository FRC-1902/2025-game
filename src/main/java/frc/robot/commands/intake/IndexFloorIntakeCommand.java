// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndexFloorIntakeCommand extends Command {
  private final FloorIntakeSubsystem floorIntakeSubsystem;
  private final EndEffectorSubsystem endEffectorSubsystem;
  /** Creates a new IndexFloorIntakeCommand. */
  public IndexFloorIntakeCommand(FloorIntakeSubsystem floorIntakeSubsystem, EndEffectorSubsystem endEffectorSubsystem){
    this.floorIntakeSubsystem = floorIntakeSubsystem; 
    this.endEffectorSubsystem = endEffectorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(floorIntakeSubsystem, endEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    floorIntakeSubsystem.setSpeed(0.2); // todo: find indexing speed
    endEffectorSubsystem.setSpeed(0.2); // todo: find indexing speed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    floorIntakeSubsystem.setSpeed(0);
    endEffectorSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // XXX: may not want front piece sensor true check here, but it probably should be
    return !floorIntakeSubsystem.pieceSensorActive() && !endEffectorSubsystem.isBackPieceSensorActive() && endEffectorSubsystem.isFrontPieceSensorActive();
  }
}
