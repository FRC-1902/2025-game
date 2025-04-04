package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.WaypointType;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveBase swerve;

  private final SwerveBase.SwerveInputs inputs = new SwerveBase.SwerveInputs();
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private int selectedReefWaypointIndex = 0;

  /**
   * Creates a SwerveSubsystem with a vision system and swerve base.
   * @param swerve
   */
  public SwerveSubsystem(SwerveBase swerve) {
    this.swerve = swerve;

    try {
      this.aprilTagFieldLayout = FieldConstants.aprilTagLayout;
    } catch (Exception e) {
      throw new RuntimeException("Failed to load AprilTag field layout", e);
    }

    swerve.setupPathPlanner(this);
    
  }

  /**
   * Updates the vision and swerve
   */
  @Override
  public void periodic() {
    swerve.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
    
    // TODO: this is temp
    SmartDashboard.putNumber("Swerve/Velocity", Math.sqrt(Math.pow(swerve.getRobotVelocity().vxMetersPerSecond, 2) + Math.pow(swerve.getRobotVelocity().vyMetersPerSecond, 2)));
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
    Pose2d visionRobotPoseMeters,
    double timestampSeconds,
    Matrix<N3, N1> visionMeasurementStdDevs
    ) {
    swerve.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  // /**
  //  * Get the distance to the speaker.
  //  *
  //  * @return Distance to speaker in meters.
  //  */
  // public double getDistanceToSpeaker() {
  // 	int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
  // 	Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
  // 	return getPose().getTranslation().getDistance(speakerAprilTagPose.toPose2d().getTranslation());
  // }

  // /**
  //  * Get the yaw to aim at the speaker.
  //  *
  //  * @return {@link Rotation2d} of which you need to achieve.
  //  */
  // public Rotation2d getSpeakerYaw() {
  // 	int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
  // 	Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
  // 	Translation2d relativeTrl =
  // 			speakerAprilTagPose.toPose2d().relativeTo(getPose()).getTranslation();
  // 	return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(getHeading());
  // }

  // /**
  //  * Get the distance to the AMP.
  //  *
  //  * @return Distance to AMP in meters.
  //  */
  // public double getDistanceToAmp() {
  // 	int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 6 : 5;
  // 	Pose3d ampAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
  // 	return getPose().getTranslation().getDistance(ampAprilTagPose.toPose2d().getTranslation());
  // }

  // /**
  //  * Get the yaw to aim at the AMP.
  //  *
  //  * @return {@link Rotation2d} of which you need to achieve.
  //  */
  // public Rotation2d getAmpYaw() {
  // 	int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 6 : 5;
  // 	Pose3d ampAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
  // 	Translation2d relativeTrl = ampAprilTagPose.toPose2d().relativeTo(getPose()).getTranslation();
  // 	return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(getHeading());
  // }

  /**
   * Get the yaw to aim at a specific AprilTag ID.
   *
   * @param tagId The ID of the AprilTag to aim at
   * @return {@link Rotation2d} of which you need to achieve, or null if tag not found
   */
  public Rotation2d getAprilTagYaw(int tagId) {
    var tagPose = aprilTagFieldLayout.getTagPose(tagId);
    if (tagPose.isEmpty()) {
      return null;
    }

    Translation2d relativeTrl = tagPose.get().toPose2d().relativeTo(getPose()).getTranslation();
    return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(getHeading());
  }

  /**
   * Get the distance to a specific AprilTag ID.
   *
   * @param tagId The ID of the AprilTag to measure distance to
   * @return Distance to tag in meters, or -1 if tag not found
   */
  public double getDistanceToAprilTag(int tagId) {
    var tagPose = aprilTagFieldLayout.getTagPose(tagId);
    if (tagPose.isEmpty()) {
      return -1;
    }

    return getPose().getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  // TODO: Tune max speeds if needed
  public Command driveToPose(Pose2d pose) {
    PathConstraints constraints =
      new PathConstraints(
        Constants.Swerve.AUTO_MAX_SPEED, 
        Constants.Swerve.AUTO_MAX_ACCELERATION, 
        Constants.Swerve.AUTO_MAX_ROTATION_SPEED.getRadians(), 
        Constants.Swerve.AUTO_MAX_ROTATION_SPEED.getRadians()
      );

    return AutoBuilder.pathfindToPose(pose, constraints, edu.wpi.first.units.Units.MetersPerSecond.of(0));
  }

  public Command getFollowPathCommand(String pathName) {
    PathPlannerPath path;
    try {
        path = PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
        DataLogManager.log("Failed to load path: " + pathName);
        return new InstantCommand();
    }

    return AutoBuilder.followPath(path);
  }

  public Command getPathfindToPathCommand(String pathName) {
    PathPlannerPath path;
    PathConstraints constraints =
    new PathConstraints(
      Constants.Swerve.AUTO_MAX_SPEED, 
      Constants.Swerve.AUTO_MAX_ACCELERATION, 
      Constants.Swerve.AUTO_MAX_ROTATION_SPEED.getRadians(), 
      Constants.Swerve.AUTO_MAX_ROTATION_SPEED.getRadians() 
    );
    try {
      path = PathPlannerPath.fromPathFile(pathName);
      
      return AutoBuilder.pathfindThenFollowPath(path, constraints);
    } catch (Exception e) {
      DataLogManager.log("ERROR: Failed to load path: " + pathName);
      return new InstantCommand();
    }
  }

  public Pose2d allianceFlip(Pose2d pose) {
    if (!isRedAlliance()) return pose;
    
    return new Pose2d(
      new Translation2d(
        FieldConstants.LENGTH - pose.getX(),
        FieldConstants.WIDTH - pose.getY()
      ),
      pose.getRotation().plus(new Rotation2d(Math.PI))
    );
  }

  public Pose2d[] allianceFlip(Pose2d[] poses) {
    if (!isRedAlliance()) return poses;
    
    Pose2d[] flippedPoses = new Pose2d[poses.length];
    for (int i = 0; i < poses.length; i++) {
        flippedPoses[i] = allianceFlip(poses[i]);
    }
    return flippedPoses;
  }

  /**
   * Gets the total number of defined REEF waypoints.
   * Assumes the offset doesn't change the number of waypoints.
   * @return The count of reef waypoints.
   */
  public int getReefWaypointCount() {
    // Use offset 0 just to get the array for its length
    Pose2d[] waypoints = FieldConstants.WAYPOINTS.getReefPositions(0);
    return waypoints != null ? waypoints.length : 0;
  }

  /**
   * Increments the selected reef waypoint index, wrapping around.
   */
  public void incrementReefWaypointIndex() {
      int count = getReefWaypointCount();
      if (count > 0) {
          selectedReefWaypointIndex = (selectedReefWaypointIndex + 1) % count;
          System.out.println("Incremented Reef Index to: " + selectedReefWaypointIndex); // Optional logging
      }
  }

  /**
   * Decrements the selected reef waypoint index, wrapping around.
   */
  public void decrementReefWaypointIndex() {
      int count = getReefWaypointCount();
      if (count > 0) {
          selectedReefWaypointIndex = (selectedReefWaypointIndex - 1 + count) % count;
           System.out.println("Decremented Reef Index to: " + selectedReefWaypointIndex); // Optional logging
      }
  }

  /**
   * Gets the selected reef waypoint index.
   * @return The current index.
   */
  public int getSelectedReefWaypointIndex() {
      return selectedReefWaypointIndex;
  }

  /**
   * Gets the Pose2d of the reef waypoint at the specified index, applies offset, and alliance flip.
   * @param index The index of the desired reef waypoint.
   * @param offset The offset distance to apply.
   * @return The Pose2d of the waypoint, or null if the index is invalid.
   */
  public Pose2d getReefWaypointByIndex(int index, double offset) {
      Pose2d[] reefWaypoints = FieldConstants.WAYPOINTS.getReefPositions(offset); // Get waypoints with offset
      if (reefWaypoints == null || index < 0 || index >= reefWaypoints.length) {
          DriverStation.reportError("Invalid Reef Waypoint Index: " + index, false);
          return null; // Or return a default pose, or the 0th index
      }
      // Apply alliance flip to the specific waypoint
      return allianceFlip(reefWaypoints[index]);
  }

  /**
   * Finds the closest waypoint of the specified type.
   *
   * @param type WaypointType
   * @return closest waypoint of the specified type
   */
  public Pose2d getWaypoint(WaypointType type, double offset) {
    Translation2d robotTranslation = swerve.getPose().getTranslation();
    Pose2d[] waypoints = null;
  
    switch (type) {
      case REEF:
        return getReefWaypointByIndex(selectedReefWaypointIndex, offset);
      case TROUGH:
        waypoints = allianceFlip(FieldConstants.WAYPOINTS.getTroughPositions());
        break;
      case BARGE:
        waypoints = allianceFlip(FieldConstants.WAYPOINTS.BARGE);
        break;
      case PROCESSOR:
        return allianceFlip(FieldConstants.WAYPOINTS.PROCESSOR);
      default:
        return null; // No waypoint specified
    }
  
    if (waypoints == null) {
      return null;
    }
  
    Pose2d targetWaypoint = null;
    double closestDistance = Double.MAX_VALUE;
  
    for (Pose2d waypoint : waypoints) {
      double distance = robotTranslation.getDistance(waypoint.getTranslation());
      if (distance < closestDistance) {
        closestDistance = distance;
        targetWaypoint = waypoint;
      }
    }

    return targetWaypoint;
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerve.drive(translation, rotation, fieldRelative);
  }

  public void drive(ChassisSpeeds velocity) {
    swerve.drive(velocity);
  }

  public SwerveDriveKinematics getKinematics() {
    return swerve.getKinematics();
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerve.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose() {
    return swerve.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerve.setChassisSpeeds(chassisSpeeds);
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerve.getSwerveController().getTargetSpeeds( 
      scaledInputs.getX(),
      scaledInputs.getY(),
      headingX, // TODO: look at me more
      headingY,
      getHeading().getRadians(),
      Constants.Swerve.MAX_SPEED
    );
  }

  public SwerveDrive getSwerveDrive()
  {
    return (SwerveDrive) swerve;
  }

  public void postTrajectory(Trajectory trajectory) {
    swerve.postTrajectory(trajectory);
  }

  public void zeroGyro() {
    swerve.zeroGyro();
  }

  /**
   * Zero the gyro and set the robot's pose based on the alliance color.
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
    } else {
      zeroGyro();
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    }
  }

  /**
   * Defaults to Blue Alliance
   * @return True if the robot is on the red alliance
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * Set the motor brake mode.
   * @param brake
   */
  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  /**
   * Get the robot's heading.
   * @return {@link Rotation2d} of the robot's heading.
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerve.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerve.getRobotVelocity();
  }

  public SwerveController getSwerveController() {
    return swerve.getSwerveController();
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerve.getSwerveDriveConfiguration();
  }

  public void lock() {
    swerve.lock();
  }

  public Rotation2d getPitch() {
    return swerve.getPitch();
  }

  private double getMaximumVelocity() {
    return Constants.Swerve.MAX_SPEED;
  }

  public double getMaximumAngularVelocity() {
    return getSwerveController().config.maxAngularVelocity;
  }
}
