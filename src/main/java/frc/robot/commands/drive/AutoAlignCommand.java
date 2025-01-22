package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

public class AutoAlignCommand extends Command {
    private Command pathfindingCommand;
    
        /**
         * Creates an AutoAlignCommand to align the robot by pathfinding and following a predefined path.
         *
         * @param swerve The swerve subsystem
         * @param pathName The name of the predefined path file
         */
        public AutoAlignCommand(SwerveSubsystem swerve) {
            PathPlannerPath path = null;

            Pose2d closestPoint = findClosestPoint(swerve.getPose());
    
            try {
                // Load the predefined path
                path = PathPlannerPath.fromPathFile("BA"); // Adjust to match your naming scheme
            } catch (Exception e) {
                e.printStackTrace();
                System.err.println("Failed to load path: " + closestPoint);
                cancelCommand(); // Cancel the command if an error occurs
            }
    
            PathConstraints constraints = new PathConstraints(
                Constants.Swerve.MAX_SPEED, // Max velocity (m/s)
                Constants.Swerve.MAX_SPEED, // Max acceleration (m/s^2)
                Constants.Swerve.MAX_ROTATION_SPEED.getRadians(), // Max angular velocity (rad/s)
                Constants.Swerve.MAX_ROTATION_SPEED.getRadians() // Max angular acceleration (rad/s^2)
            );
    
            if (path != null) {
                pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
            } else {
                pathfindingCommand = null; // Null if path loading failed
            }
    
            addRequirements(swerve);
        }
    
        @Override
        public void initialize() {
            if (pathfindingCommand != null) {
                pathfindingCommand.initialize();
            } else {
                System.err.println("Pathfinding command not initialized due to path loading failure.");
            }
        }
    
        @Override
        public void execute() {
            if (pathfindingCommand != null) {
                pathfindingCommand.execute();
            }
        }
    
        @Override
        public void end(boolean interrupted) {
            if (pathfindingCommand != null) {
                pathfindingCommand.end(interrupted);
            }
        }
    
        @Override
        public boolean isFinished() {
            return pathfindingCommand == null || pathfindingCommand.isFinished();
        }
    
        /**
         * Cancels the command by setting the required fields to null.
         */
        private void cancelCommand() {
            pathfindingCommand = null;
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
