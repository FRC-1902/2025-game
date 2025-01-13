package frc.robot;

import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
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

    ControllerSubsystem controllerSubsystem = new ControllerSubsystem();
 
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "neo"));

    DriveCommand closedDrive = new DriveCommand(
        drivebase,
        () -> -MathUtil.applyDeadband(controllerSubsystem.getCommandController(ControllerName.DRIVE).getLeftY(), Constants.Controllers.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(controllerSubsystem.getCommandController(ControllerName.DRIVE).getLeftX(), Constants.Controllers.DEADBAND),
        () -> -MathUtil.applyDeadband(controllerSubsystem.getCommandController(ControllerName.DRIVE).getRightX(), Constants.Controllers.RIGHT_X_DEADBAND),
        controllerSubsystem.getCommandController(ControllerName.DRIVE).getHID()::getYButtonPressed,
        controllerSubsystem.getCommandController(ControllerName.DRIVE).getHID()::getAButtonPressed,
        controllerSubsystem.getCommandController(ControllerName.DRIVE).getHID()::getXButtonPressed,
        controllerSubsystem.getCommandController(ControllerName.DRIVE).getHID()::getBButtonPressed
    );

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
        drivebase.getSwerveDrive(),
        () -> controllerSubsystem.getCommandController(ControllerName.DRIVE).getLeftY() * -1,
        () -> controllerSubsystem.getCommandController(ControllerName.DRIVE).getLeftX() * -1 )
        .withControllerRotationAxis(controllerSubsystem.getCommandController(ControllerName.DRIVE)::getRightX)
        .deadband(Constants.Controllers.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
        controllerSubsystem.getCommandController(ControllerName.DRIVE)::getRightX,
        controllerSubsystem.getCommandController(ControllerName.DRIVE)::getRightY)
        .headingWhile(true);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

    SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(
        drivebase.getSwerveDrive(),
        () -> -controllerSubsystem.getCommandController(ControllerName.DRIVE).getLeftY(),
        () -> -controllerSubsystem.getCommandController(ControllerName.DRIVE).getLeftX()
        )
            .withControllerRotationAxis(() -> controllerSubsystem.getCommandController(ControllerName.DRIVE).getRawAxis(2))
            .deadband(Constants.Controller.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
        .withControllerHeadingAxis(
            () -> Math.sin( controllerSubsystem.getCommandController(ControllerName.DRIVE).getRawAxis(2) * Math.PI) * (Math.PI * 2),
            () -> Math.cos( controllerSubsystem.getCommandController(ControllerName.DRIVE).getRawAxis(2) * Math.PI) * (Math.PI * 2))
        .headingWhile(true);

    Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

    Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

    public RobotContainer() {
        vision = new VisionSubsystem(Robot.isSimulation() ? new VisionSim() : new VisionReal());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return drivebase.getAutonomousCommand("New Auto");
    }

    public void setMotorBrake(boolean brake)
    {
        drivebase.setMotorBrake(brake);
    }
}