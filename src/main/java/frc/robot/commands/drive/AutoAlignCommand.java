package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

public class AutoAlignCommand extends Command {
    private final Command pathfindingCommand;

    Pose2d targetPose = new Pose2d(30.1, 10.1, Rotation2d.fromDegrees(175));

    /**
     * Creates an AutoAlignCommand to align the robot to a target pose.
     *
     * @param swerve The swerve subsystem
     * @param targetPose The target pose to align to
     */
    public AutoAlignCommand(SwerveSubsystem swerve, Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
            Constants.Swerve.MAX_SPEED, // Max velocity (m/s)
            Constants.Swerve.MAX_SPEED, // Max acceleration (m/s^2)
            Constants.Swerve.MAX_ROTATION_SPEED.getRadians(), // Max angular velocity (rad/s)
            Constants.Swerve.MAX_ROTATION_SPEED.getRadians()  // Max angular acceleration (rad/s^2)
        );

        pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0 // Goal end velocity (m/s)
        );
        addRequirements(swerve);
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
}
