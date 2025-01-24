package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            Constants.Swerve.MAX_ACCELERATION, // Max velocity (m/s)
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
                
                double kP = 10; // TODO: Tune

                swerve.drive(((targetTranslation.minus(currentTranslation)).times(kP)), targetPose.getRotation().getRotations(), true);
            }

            @Override
            public void end(boolean interrupted) {
            }

            @Override
            public boolean isFinished() {
                return swerve.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.001;
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
