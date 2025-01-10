package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

// TODO: Change to actaull cameras
public interface VisionBase {
	@AutoLog
	public static class VisionInputs {
		public double timestamp;

		public boolean ArducamOne = false;
		public boolean ArducamTwo = false;
		public boolean ArducamThree = false;
		public boolean ArducamFour = false;

		public double ArducamOneLatencyMS = 0.0;
		public double ArducamTwoLatencyMS = 0.0;
		public double ArducamThreeLatencyMS = 0.0;
		public double ArducamFourLatencyMS = 0.0;

        // IDK what this does/works, I think it somehow gets the best target ID from somewhere
		public double ArducamOneBestTargetID = -1.0;
		public double ArducamTwoBestTargetID = -1.0;
		public double ArducamThreeBestTargetID = -1.0;
		public double ArducamFourBestTargetID = -1.0;

		public Pose3d[] visibleTagPoses = new Pose3d[0];
	}

	public void updateInputs(VisionInputs inputs);

	public void updatePoseEstimation(Pose2d currentPose);

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose();
}