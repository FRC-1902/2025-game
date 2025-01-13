package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveController;
import swervelib.parser.SwerveDriveConfiguration;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveBase swerve;
    private final VisionSubsystem vision;

    private final SwerveBase.SwerveInputs inputs = new SwerveBase.SwerveInputs();
    private final AprilTagFieldLayout aprilTagFieldLayout;

    /**
     * Creates a SwerveSubsystem with a vision system and swerve base.
     * @param vision
     * @param swerve
     */
    public SwerveSubsystem(VisionSubsystem vision, SwerveBase swerve) {
        this.swerve = swerve;
        this.vision = vision;

        try {
            this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); // TODO: Update to new field layout
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

        // Update vision with current swerve pose
        vision.updatePoseEstimation(getPose());

        // Add new vision measurements to the swerve drive odometry if its available
        if (vision.getEstimatedGlobalPose().isPresent()) {
            swerve.addVisionMeasurement(
                vision.getEstimatedGlobalPose().get().estimatedPose.toPose2d(),
                vision.getEstimatedGlobalPose().get().timestampSeconds);

            // Record estimated pose output
            Logger.recordOutput(
                "Vision/CurrentEstimatedPose", vision.getEstimatedGlobalPose().get().estimatedPose);
        }

        // Convert Pose2d to Pose3d for camera position transformation
        Pose3d robotPose3d = new Pose3d(inputs.robotPose);

        // Transform camera positions to global coordinate system
        Pose3d[] globalCameraPositions = new Pose3d[Constants.Vision.CAMERA_POSITIONS.length];
        for (int i = 0; i < Constants.Vision.CAMERA_POSITIONS.length; i++) {
            globalCameraPositions[i] = robotPose3d.transformBy(
                new Transform3d(
                    Constants.Vision.CAMERA_POSITIONS[i].getTranslation(),
                    Constants.Vision.CAMERA_POSITIONS[i].getRotation()
                )
            );
        }

        // Record camera positions in global coordinate system
        // In AScope set camera positions to cone object, and the pointy end is where the camera is
        // looking at
        // If you want to calibrate the postion set camera positions to transform object instead
        Logger.recordOutput("CameraPositions", globalCameraPositions);

        // Update inputs
        swerve.updateInputs(inputs);

        Logger.processInputs("Swerve", inputs);
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
     * Get the path follower with events.
     *
     * @param pathName PathPlanner path name.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command getAutonomousCommand(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints =
            new PathConstraints(
                getMaximumVelocity(), 
                4.0, 
                getMaximumAngularVelocity(), 
                Units.degreesToRadians(720)
            );

        return AutoBuilder.pathfindToPose(
                pose, constraints, edu.wpi.first.units.Units.MetersPerSecond.of(0));
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
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            zeroGyro();
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
