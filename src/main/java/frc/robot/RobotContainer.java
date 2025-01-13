package frc.robot;

import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.subsystems.swerve.SwerveReal;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.subsystems.vision.VisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

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

    private final SwerveSubsystem swerve;
	private final VisionSubsystem vision;
    ControllerSubsystem controllers;

    public RobotContainer() {
        controllers = new ControllerSubsystem();

        swerve = new SwerveSubsystem(vision, new SwerveReal(new File(Filesystem.getDeployDirectory(), "swerve")));
        vision = new VisionSubsystem(Robot.isSimulation() ? new VisionSim() : new VisionReal());
    }

    DriveCommand closedDrive = new DriveCommand(
        swerve,
        () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getLeftY(), Constants.Controllers.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getLeftX(), Constants.Controllers.DEADBAND),
        () -> -MathUtil.applyDeadband(controllers.getCommandController(ControllerName.DRIVE).getRightX(), Constants.Controllers.RIGHT_X_DEADBAND),
        controllers.getCommandController(ControllerName.DRIVE).getHID()::getYButtonPressed,
        controllers.getCommandController(ControllerName.DRIVE).getHID()::getAButtonPressed,
        controllers.getCommandController(ControllerName.DRIVE).getHID()::getXButtonPressed,
        controllers.getCommandController(ControllerName.DRIVE).getHID()::getBButtonPressed
    );


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return swerve.getAutonomousCommand("New Auto");
    }

    public void setMotorBrake(boolean brake)
    {
        swerve.setMotorBrake(brake);
    }
}