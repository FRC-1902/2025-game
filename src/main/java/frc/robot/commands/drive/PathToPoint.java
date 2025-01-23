package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

public class PathToPoint extends SequentialCommandGroup {

    /**
     * Creates a SequentialCommandGroup to navigate to a target pose and refine alignment.
     *
     * @param swerve The swerve subsystem
     * @param targetPose The target pose to align to
     */
    public PathToPoint(SwerveSubsystem swerve, Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
            Constants.Swerve.MAX_SPEED, // Max velocity (m/s)
            Constants.Swerve.MAX_SPEED, // Max velocity (m/s)
            Constants.Swerve.MAX_ROTATION_SPEED.getRadians(), // Max acceleration (m/s^2)
            Constants.Swerve.MAX_ROTATION_SPEED.getRadians() // Max acceleration (m/s^2)
        );

        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0 // Goal end velocity (m/s)
        );

        Command alignmentCommand = new Command() {
            @Override
            public void initialize() {}

            @Override
            public void execute() {
                Translation2d currentTranslation = swerve.getPose().getTranslation();
                Translation2d targetTranslation = targetPose.getTranslation();

                Translation2d error = targetTranslation.minus(currentTranslation);
                double distance = error.getNorm();

                // Apply a simple P-controller for drive velocities
                double kP = 1.0; // Tune this constant as needed
                Translation2d velocity = error.times(kP / Math.max(distance, 0.1)); // Scale by distance, avoid div by 0

                swerve.drive(velocity, targetPose.getRotation().getRadians(), true);
            }

            @Override
            public void end(boolean interrupted) {
            }

            @Override
            public boolean isFinished() {
                double distanceThreshold = 0.01; // 5 cm
                return swerve.getPose().getTranslation().getDistance(targetPose.getTranslation()) < distanceThreshold;
            }
        };

        // Add commands to the sequential group
        addCommands(
            pathfindingCommand,
            alignmentCommand
        );

        addRequirements(swerve);
    }
}
