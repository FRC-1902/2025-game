package frc.robot.commands.drive;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class PathToFollowPath {
  /**
   * Creates a command to pathfind to and follow the specified path
   * 
   * @param pathName Name of the path file to load and follow
   * @param swerve SwerveSubsystem to use for driving
   * @return A command that pathfinds to and follows the specified path
   */
  public static Command path(String pathName, SwerveSubsystem swerve) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      
      PathConstraints constraints = new PathConstraints(
        Constants.Swerve.AUTO_MAX_SPEED, 
        Constants.Swerve.AUTO_MAX_ACCELERATION, 
        Constants.Swerve.AUTO_MAX_ROTATION_SPEED.getRotations(), 
        Constants.Swerve.AUTO_MAX_ROTATION_SPEED.getRotations()
      );
      
      return AutoBuilder.pathfindThenFollowPath(path, constraints);

    } catch (Exception e) {
      DataLogManager.log("ERROR: Failed to load path: " + pathName);
      e.printStackTrace();
      return new InstantCommand();
    }
  }
}
