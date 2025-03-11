// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FloorIntake;
import frc.robot.subsystems.vision.DetectionSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class ObjectAlign extends Command {

  private final DetectionSubsystem detectionSubsystem;
  private final SwerveSubsystem swerve;
  private final FloorIntakeSubsystem floorIntakeSubsystem;

  /*
  * Moves backwards and centers object when detected
  */
  public ObjectAlign(DetectionSubsystem detectionSubsystem, SwerveSubsystem swerve, FloorIntakeSubsystem floorIntakeSubsystem) {
    this.detectionSubsystem = detectionSubsystem;
    this.swerve = swerve;
    this.floorIntakeSubsystem = floorIntakeSubsystem;

    addRequirements(detectionSubsystem, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn = -detectionSubsystem.getTargetYaw().getRadians() * Constants.Swerve.OBJECT_TURN_KP;
    if (detectionSubsystem.isTargetVisible()) {
      swerve.drive(new Translation2d(-1,0), turn, false);
    } else {
      swerve.drive(new Translation2d(0,0), 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0,0), 0, false);    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return floorIntakeSubsystem.pieceSensorActive(); //!detectionSubsystem.isTargetVisible()
  }
}
