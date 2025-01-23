package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

public class toPoseCommand extends Command {
    private Command pathfindingCommand;
    
        /**
         * Creates an AutoAlignCommand to align the robot by pathfinding and following a predefined path.
         *
         * @param swerve The swerve subsystem
         * @param pathName The name of the predefined path file
         */
        public toPoseCommand(SwerveSubsystem swerve) {

          PathPlannerPath path = null;
          try{
            // Load the path you want to follow using its name in the GUI
            path = PathPlannerPath.fromPathFile("BA");
  
            // Create a path following command using AutoBuilder. This will also trigger event markers.
          } catch (Exception e) {
              DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
          }
            addRequirements(swerve);
            pathfindingCommand = AutoBuilder.followPath(path);

        }

        @Override
        public void initialize() {
          pathfindingCommand.initialize();
        }
    
        @Override
        public void execute() {
          pathfindingCommand.execute();

        }
    
        @Override
        public void end(boolean interrupted) {
          pathfindingCommand.end(interrupted);
        }
    
        @Override
        public boolean isFinished() {
            return pathfindingCommand.isFinished();
        }
    
       
    /**
     * Finds the closest alignment point to the robot's current position.
     *
     * @param currentPose The robot's current pose
     * @return The closest alignment point
     */
    private Pose2d findClosestPoint(Pose2d currentPose) {
      double minDistance = Double.MAX_VALUE;
      Pose2d closestPoint = null;

        for (Pose2d point : Constants.Vision.WAYPOINTS) {
            double distance = currentPose.getTranslation().getDistance(point.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closestPoint = point;
            }
        }

        return closestPoint;
      }
}
