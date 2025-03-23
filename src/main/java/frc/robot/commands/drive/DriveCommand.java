package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveCommand extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX, vY, heading;
  private double rotationMultiplier;
  private IntSupplier dpad;


  /**
   * Creates a DriveCommand
   * @param swerve
   * @param vX
   * @param vY
   * @param heading
   */
  public DriveCommand(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading, IntSupplier dpad) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;
    this.dpad = dpad;

    rotationMultiplier = 0.40;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // take away cubic scaling!!!!!
    // Apply cubic scaling to x and y velocities
    Translation2d trans = new Translation2d(scaleInputsOne(vX.getAsDouble()), scaleInputsOne(vY.getAsDouble())).times(Constants.Swerve.MAX_SPEED).times(1);
    double xVelocity = trans.getX(); 
    double yVelocity = trans.getY();

    // Apply alliance-based inversions
    var alliance = DriverStation.getAlliance();

    // Additional alliance-based inversions
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      // Invert x and y velocities for blue alliance
      xVelocity *= -1;
      yVelocity *= -1;
    }

    if (dpad.getAsInt() == 90) {
      rotationMultiplier = 1.0;
    } else {
      rotationMultiplier = 0.40;
    }
    double rotationVelocity = heading.getAsDouble() * Constants.Swerve.MAX_ROTATION_SPEED.getRadians() * rotationMultiplier; // TODO: change speed cap

    SmartDashboard.putNumber("swerve/Target Velocity", Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2)));

    // Create field-relative ChassisSpeeds
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xVelocity, 
      yVelocity, 
      rotationVelocity, 
      swerve.getHeading()
    );

    // Drive using field-relative speeds
    swerve.drive(fieldRelativeSpeeds);
  }

  @Override
  public void end(boolean interrupted) {} 

  @Override
  public boolean isFinished() {
    return false;
  }

  // Functions specifically for controller inputs
  
  // y = .8 * x + .2 * x^3
  public double scaleInputsOne(double input){
    return (.8 * input + .2 * Math.pow(input, 3));
  }

  // y = .6 * x + .4 * x^3
  public double scaleInputsTwo(double input){
    return (.6 * input + .4 * Math.pow(input, 3));
  }

  // y = .4 * x + .6 * x^3 
  public double scaleInputsThree(double input){
    return (.4 * input + .6 * Math.pow(input, 3));
  }

  // y = .2 * x + .8 * x^3
  public double scaleInputsFour(double input){
    return (.2 * input + .8 * Math.pow(input, 3));
  }
}
