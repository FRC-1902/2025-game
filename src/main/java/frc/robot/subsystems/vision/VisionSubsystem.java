package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class VisionSubsystem extends SubsystemBase {
	private final VisionBase vision;
	private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

	public VisionSubsystem(VisionBase vision) {
		this.vision = vision;
	}

	@Override
	public void periodic() {
		// Update inputs
		vision.updateInputs(inputs);

		// Process inputs
		Logger.processInputs("Vision", inputs);
	}

	// This will be called by the Swerve Drive subsystem to update the estimated pose.
	public void updatePoseEstimation(Pose2d currentPose) {
		vision.updatePoseEstimation(currentPose);
	}

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
		return vision.getEstimatedGlobalPose();
	}

	// TODO: Change to actual cameras
	public boolean hasTargets() {
		return inputs.arducamOne || inputs.arducamTwo || inputs.arducamThree;
	}
}
