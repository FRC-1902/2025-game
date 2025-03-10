package frc.robot.commands.drive;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class PathToFollowPath extends Command {
  private PathConstraints constraints;
  private String pathName;
  private Command pathCommand;

  /** Creates a new PathToWaypoint. */
  public PathToFollowPath(String pathName, SwerveSubsystem swerve) {
    this.pathName = pathName;

    constraints = new PathConstraints(
      Constants.Swerve.AUTO_MAX_SPEED, 
      Constants.Swerve.AUTO_MAX_ACCELERATION, 
      Constants.Swerve.AUTO_MAX_ROTATION_SPEED.getRotations(), 
      Constants.Swerve.AUTO_MAX_ROTATION_SPEED.getRotations()
    );

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile(pathName);
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }
    
    if (path != null) {
      pathCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
      pathCommand.schedule();
    } else {
      System.err.println("Failed to load path: " + pathName);
    }
  }

  @Override
  public void execute() {}
  
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return pathCommand == null || !pathCommand.isScheduled();
  }
}
