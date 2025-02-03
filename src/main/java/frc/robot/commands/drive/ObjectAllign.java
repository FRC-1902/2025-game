// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.Axis;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.Constants;
import frc.robot.subsystems.vision.DetectionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ObjectAllign extends Command {
  private final ControllerSubsystem controller; 
  private final DetectionSubsystem detectionSubsystem;
  public double turn;
  /** Creates a new ObjectAllign. */
  public ObjectAllign(ControllerSubsystem controller, DetectionSubsystem detectionSubsystem) {
    this.controller = controller; 
    this.detectionSubsystem = detectionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(detectionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(detectionSubsystem.targetVisible){
      turn = -1.0 * detectionSubsystem.targetYaw * Constants.Swerve.TURN_KP * Constants.Swerve.MAX_ROTATION_SPEED.getRadians();
    }
    turn = -controller.get(ControllerName.DRIVE, Axis.RX) * Constants.Swerve.MAX_ROTATION_SPEED.getRadians();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
