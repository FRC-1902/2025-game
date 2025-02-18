package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class CancelPathCommand extends Command {

  private Command pathCommand;
  private SwerveSubsystem swerve;
  private String pathName;

  /** Creates a new PathToWaypoint. */
  public CancelPathCommand(SwerveSubsystem swerve, String pathName) {
    this.pathName = pathName;
  }

  @Override
  public void initialize() {
    pathCommand = swerve.getFollowPathCommand(pathName);
    pathCommand.initialize();
  }

  @Override
  public void execute() {
    pathCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    // TODO: ends prematurely
    return pathCommand.isFinished();
  }
}
