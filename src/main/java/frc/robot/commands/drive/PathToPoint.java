package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class PathToPoint extends SequentialCommandGroup {

    /**
     * Creates a SequentialCommandGroup to navigate to a target pose and refine alignment.
     *
     * @param swerve The swerve subsystem
     */
    public PathToPoint(SwerveSubsystem swerve) {
        // Calculate a targetPose by picking the closest known waypoint
        Pose2d targetPose = closestPoint(swerve);

        // Build path constraints for PathPlanner
        PathConstraints constraints = new PathConstraints(
            Constants.Swerve.MAX_SPEED,                   // Max linear velocity (m/s)
            Constants.Swerve.MAX_ACCELERATION,            // Max linear acceleration (m/s^2)
            Constants.Swerve.MAX_ROTATION_SPEED.getRadians(), // Max rotational velocity (rad/s)
            Constants.Swerve.MAX_ROTATION_SPEED.getRadians()  // Max rotational acceleration (rad/s^2)
        );

        // Create a pathfinding command to drive to the target pose
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0 // Desired end velocity (m/s)
        );

        // Create a custom alignment command to fine-tune final position/orientation
        Command alignmentCommand = new Command() {
            // Capture the target translation & rotation for alignment
            private final Translation2d targetTranslation = targetPose.getTranslation();
            private final Rotation2d targetRotation = targetPose.getRotation();

            @Override
            public void initialize() {
                // Runs once when this command starts
            }

            @Override
            public void execute() {
                // Called every 20ms or so while this command is scheduled

                // Current robot pose
                Translation2d currentTranslation = swerve.getPose().getTranslation();
                Rotation2d currentRotation = swerve.getPose().getRotation();

                // Simple P-controllers for translation and rotation
                double velocitykP = 10.0; 
                double rotationkP = 10.0;

                Translation2d velocity =
                    targetTranslation.minus(currentTranslation).times(velocitykP);

                // Convert rotational difference to a Rotation2d, multiply by rotationkP
                Rotation2d rotationError =
                    targetRotation.minus(currentRotation).times(rotationkP);

                // Drive with the computed velocity and rotation
                // "rotationError.getRotations()" gives you the rotation in *rotations*, 
                // but presumably you want degrees or radians. Double-check what your swerve.drive requires.
                swerve.drive(velocity, rotationError.getRotations(), true);
            }

            @Override
            public void end(boolean interrupted) {
                // Called once when this command finishes or is interrupted
            }

            @Override
            public boolean isFinished() {
                // Finish when position and orientation are close enough
                double distanceThreshold = 0.01;        // meters
                double rotationThreshold = Math.toRadians(1); // 1 degree in radians

                double currentDistance =
                    swerve.getPose().getTranslation().getDistance(targetTranslation);
                double currentRotError = Math.abs(
                    swerve.getPose().getRotation().getRadians()
                    - targetRotation.getRadians()
                );

                boolean positionReached = (currentDistance < distanceThreshold);
                boolean rotationReached = (currentRotError < rotationThreshold);

                return positionReached && rotationReached;
            }
        };

        // Add commands in sequence: follow the path first, then fine-tune alignment
        addCommands(
            pathfindingCommand,
            alignmentCommand.withTimeout(5) // 5s timeout on alignment
        );

        // Declare that this command uses the swerve subsystem
        addRequirements(swerve);
    }

    /**
     * Return the known waypoint closest to the robot's current pose.
     */
    public Pose2d closestPoint(SwerveSubsystem swerve) {
        Translation2d robotTranslation = swerve.getPose().getTranslation();
        Pose2d closestWaypoint = null;
        double closestDistance = Double.MAX_VALUE; // Large initial value

        // Display current robot position for debugging
        SmartDashboard.putString("Auto/Robot", robotTranslation.toString());

        // Check each waypoint to see if it's closer than the current best
        for (Pose2d waypoint : Constants.Auto.WAYPOINTS) {
            double distance = robotTranslation.getDistance(waypoint.getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                closestWaypoint = waypoint;
                SmartDashboard.putString("Auto/Closest_Waypoint", closestWaypoint.toString());
            }
        }

        return closestWaypoint;
    }
}
//   Pose2d targetPose = closestPoint(swerve);


//     /**
//      * Creates a SequentialCommandGroup to navigate to a target pose and refine alignment.
//      *
//      * @param swerve The swerve subsystem
//      */
//     public PathToPoint(SwerveSubsystem swerve) {

//         PathConstraints constraints = new PathConstraints(
//             Constants.Swerve.MAX_SPEED, // Max velocity (m/s)
//             Constants.Swerve.MAX_ACCELERATION, // Max velocity (m/s)
//             Constants.Swerve.MAX_ROTATION_SPEED.getRadians(), // Max acceleration (m/s^2)
//             Constants.Swerve.MAX_ROTATION_SPEED.getRadians() // Max acceleration (m/s^2)
//         );
//         Command pathfindingCommand = AutoBuilder.pathfindToPose(
//           targetPose,
//           constraints,
//           0.0 // Goal end velocity (m/s)
//         );
//         addRequirements(swerve);
//     }

//       Command alignmentCommand = new Command() {
//           Translation2d targetTranslation = targetPose.getTranslation();
//           Rotation2d targetRotation = targetPose.getRotation();

//           @Override
//           public void initialize() {
            
//           }

//           @Override
//           public void execute() {
//               Translation2d currentTranslation = swerve.getPose().getTranslation();
//               Rotation2d currentRotation = swerve.getPose().getRotation();
              
//               double velocitykP = 10; // TODO: Tune
//               double rotationkP = 10; // TODO: Tune

//               Translation2d velocity = targetTranslation.minus(currentTranslation).times(velocitykP);
//               Rotation2d rotaion = targetRotation.minus(currentRotation).times(rotationkP);

//               swerve.drive(velocity, rotaion.getRotations(), true);
//           }

//           @Override
//           public void end(boolean interrupted) {
//           }

//           @Override
//           public boolean isFinished() {
//             double distanceThreshold = 0.01;
//             double rotationThreshold = Math.toRadians(1);
//             boolean positionReached = swerve.getPose().getTranslation().getDistance(targetPose.getTranslation()) < distanceThreshold;
//             boolean rotationReached = Math.abs(swerve.getPose().getRotation().getRadians() - targetPose.getRotation().getRadians()) < rotationThreshold;
//             return positionReached && rotationReached;
//         }
//       };
      
//       // Add commands to the sequential group
//       addCommands(
//           pathfindingCommand,
//           alignmentCommand.withTimeout(5)
//       );


//     public Pose2d closestPoint(SwerveSubsystem swerve) {
//       Translation2d targetTranslation = swerve.getPose().getTranslation();
//       Pose2d closestWaypoint = null;
//       double closestDistance = 10.0;
//       SmartDashboard.putString("Auto/Robot", targetTranslation.toString());


//       // Check each waypoint to see if its closer than the current closest waypoint
//       for (Pose2d waypoint : Constants.Auto.WAYPOINTS) {
//           double distance = targetTranslation.getDistance(waypoint.getTranslation());
//           if (distance < closestDistance) {
//               closestDistance = distance;
//               closestWaypoint = waypoint;
//               SmartDashboard.putString("Auto/Closest_Waypoint", closestWaypoint.toString());
//           }
//       }
//       return closestWaypoint;
//     }
// }
