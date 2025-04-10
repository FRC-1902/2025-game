package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.WaypointType;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.ObjectDetectionSubsystem;
import frc.robot.commands.drive.AutoDriveFactory;
import frc.robot.commands.drive.SnapToWaypoint; 


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoSnapToCoral extends Command {
  private final SwerveSubsystem swerve;
  private final ObjectDetectionSubsystem objectDetection;
  private Pose2d targetPose;
  private final PIDController pidX;
  private final PIDController pidY;
  private final double distanceErrorTolerance = Units.inchesToMeters(2); // meters
  private final double rotationErrorTolerance = Math.toRadians(2); // degrees
  private final double maxVelocity = 4;
  private final double rotationkP = 5;
  private double currentDistance;
  private double currentRotError;
  private Boolean earlyExit = false;

  /**
   * Creates a command that drives to and aligns with a tracked coral with custom velocity
   * @param swerve SwerveSubsystem for movement
   * @param objectDetection ObjectDetectionSubsystem for coral tracking
   * @param maxVelocity Maximum velocity when approaching the coral
   */
  public AutoSnapToCoral(SwerveSubsystem swerve, ObjectDetectionSubsystem objectDetection) {
    this.swerve = swerve;
    this.objectDetection = objectDetection;
    this.pidX = new PIDController(2.5, 0.02, 0.001);
    this.pidY = new PIDController(2.5, 0.02, 0.001);

    pidX.reset();
    pidY.reset();

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    DataLogManager.log("SnapToTrackedCoral initialized");
    pidX.reset();
    pidY.reset();
  }

  @Override
  public void execute() {
    // Get the tracked coral object
    Pose2d coralPose = objectDetection.getTrackedObject();
    Pose2d targetPoseToUse;

    // If no coral found, stop moving
    if (coralPose == null) {
      // If no coral found, use a fallback waypoint
        targetPoseToUse = swerve.getWaypoint(FieldConstants.WaypointType.HP, 0);
        SmartDashboard.putBoolean("Vision/GoingToHold", true);
        // if (targetPoseToUse == null) {
        //   swerve.drive(new Translation2d(0, 0), 0, true);
        //   earlyExit = true;
        //   return;
        // }
    } else {
      Transform2d offset = new Transform2d(
        new Translation2d(FieldConstants.INTAKE_OFFSET, 0),
        new Rotation2d(0)
      );
      targetPoseToUse = coralPose.plus(offset);
    }
      
    targetPose = targetPoseToUse;
    
    // Calculate current position error
    Pose2d currentPose = swerve.getPose();
    currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    currentRotError = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());
    
    // Calculate velocity with PID
    double xVelocity = pidX.calculate(currentPose.getX(), targetPose.getX());
    double yVelocity = pidY.calculate(currentPose.getY(), targetPose.getY());
    
    // Cap velocity
    Translation2d velocity = new Translation2d(xVelocity, yVelocity);
    Translation2d cappedVelocity = velocity;
    
    double v = velocity.getDistance(Translation2d.kZero);
    if (v > maxVelocity) {
      cappedVelocity = cappedVelocity.times(1.0 / v).times(maxVelocity);
    }
    
    // Calculate rotation to target
    Rotation2d rotation = targetPose.getRotation().minus(currentPose.getRotation()).times(rotationkP);
    double cappedRotation = Math.max(Math.min(rotation.getRadians(), 3), -3);
    
    // Drive to target
    swerve.drive(cappedVelocity, cappedRotation, true);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, true);

    objectDetection.resetTracking();

    DataLogManager.log("SnapToTrackedCoral ended, interrupted: " + interrupted);
  }

  @Override
  public boolean isFinished() {
    // Finish when position and orientation are close enough
    boolean positionReached = (currentDistance < distanceErrorTolerance);
    boolean rotationReached = (currentRotError < rotationErrorTolerance);
    return positionReached && rotationReached || earlyExit;
  }
}
