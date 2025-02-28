package frc.robot.commands.drive;

import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class PathToWaypoint extends Command {
  private PathConstraints constraints;
  private Supplier<Pose2d> targetPose;
  private Command pathCommand;

  /** Creates a new PathToWaypoint. */
  public PathToWaypoint(Supplier<Pose2d> targetPose, SwerveSubsystem swerve) {
    this.targetPose = targetPose;

    constraints = new PathConstraints(
      Constants.Swerve.AUTO_MAX_SPEED, 
      Constants.Swerve.AUTO_MAX_ACCELERATION, 
      Constants.Swerve.AUTO_MAX_ROTATION_SPEED.getRotations(), 
      Constants.Swerve.AUTO_MAX_ROTATION_SPEED.getRotations() // TODO: Change to rotation accel
    );

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    pathCommand = AutoBuilder.pathfindToPose(targetPose.get(), constraints, 0);
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
