// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveInputStream;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  private final Swerve drivebase = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(
    drivebase,
    () -> -MathUtil.applyDeadband(driverController.getLeftY(), Constants.Controllers.LEFT_Y_DEADBAND),
    () -> -MathUtil.applyDeadband(driverController.getLeftX(), Constants.Controllers.DEADBAND),
    () -> -MathUtil.applyDeadband(driverController.getRightX(), Constants.Controllers.RIGHT_X_DEADBAND),
    driverController.getHID()::getYButtonPressed,
    driverController.getHID()::getAButtonPressed,
    driverController.getHID()::getXButtonPressed,
    driverController.getHID()::getBButtonPressed
  );

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    () -> driverController.getLeftY() * -1,
    () -> driverController.getLeftX() * -1 )
    .withControllerRotationAxis(driverController::getRightX)
    .deadband(Constants.Controllers.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
    driverController::getRightX,
    driverController::getRightY)
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
    () -> -driverController.getLeftY(),
    () -> -driverController.getLeftX()
    )
      .withControllerRotationAxis(() -> driverController.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
    .withControllerHeadingAxis(
      () -> Math.sin( driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
      () -> Math.cos( driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
    .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  public RobotContainer() {
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
