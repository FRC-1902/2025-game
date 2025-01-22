package frc.robot;

import frc.robot.commands.drive.AutoAlignCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.subsystems.swerve.SwerveReal;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.subsystems.vision.VisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private SwerveSubsystem swerve;
    private VisionSubsystem vision;
    ControllerSubsystem controllers;

    public RobotContainer() {
        controllers = new ControllerSubsystem();
        vision = new VisionSubsystem(Robot.isSimulation() ? new VisionSim() : new VisionReal());
        swerve = new SwerveSubsystem(vision, new SwerveReal(new File(Filesystem.getDeployDirectory(), "swerve")));

        DriveCommand closedDrive = new DriveCommand(
            swerve,
            () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getLeftY(), Constants.Controller.LEFT_Y_DEADBAND),
            () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getLeftX(), Constants.Controller.LEFT_Y_DEADBAND),
            () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getRightX(), Constants.Controller.RIGHT_X_DEADBAND)
        );

        swerve.setDefaultCommand(closedDrive);
        new JoystickButton(controllers.getCommandController(ControllerName.DRIVE).getHID(), XboxController.Button.kA.value)
            .onTrue(new AutoAlignCommand(swerve, Constants.Vision.WAYPOINTS[0], "BA"));
    }

 

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return swerve.getAutonomousCommand("Test");
    }
}