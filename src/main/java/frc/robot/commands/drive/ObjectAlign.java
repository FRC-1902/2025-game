// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.vision.DetectionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ObjectAlign extends Command {

  private final DetectionSubsystem detectionSubsystem;
  private final SwerveSubsystem swerve;
  /** Creates a new ObjectAllign. */
  public ObjectAlign(DetectionSubsystem detectionSubsystem, SwerveSubsystem swerve) {
    this.detectionSubsystem = detectionSubsystem;
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(detectionSubsystem, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(detectionSubsystem.isTargetVisible()){
      DataLogManager.log("Target Visible, driving");
      double turn = -detectionSubsystem.getTargetYaw().getRadians() * Constants.Swerve.OBJECT_TURN_KP;
      DataLogManager.log(Double.toString(turn));
      swerve.drive(new Translation2d(0,0), turn, true);
    }
    else{
      DataLogManager.log("No Target Visible, stationary");
      swerve.drive(new Translation2d(0,0), 0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0,0), 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!detectionSubsystem.isTargetVisible()) {
      DataLogManager.log("Piece not visible");
      return true;
    }

    return Math.abs(swerve.getPose().getRotation().getRadians() - detectionSubsystem.getTargetYaw().getRadians()) <= 0.001;
  }
}
