package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class VisionSubsystem extends SubsystemBase {
	private final VisionBase io;
	private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

	public VisionSubsystem(VisionBase io) {
		this.io = io;
	}

	@Override
	public void periodic() {

		// Update inputs
		io.updateInputs(inputs);

		// Process inputs
		Logger.processInputs("Vision", inputs);
	}

	// This will be called by the Swerve Drive subsystem to update the estimated pose.
	public void updatePoseEstimation(Pose2d currentPose) {
		io.updatePoseEstimation(currentPose);
	}

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
		return io.getEstimatedGlobalPose();
	}

    // TODO: Change to actual cameras
	public boolean hasTargets() {
		return inputs.arducamOne || inputs.arducamTwo || inputs.arducamThree || inputs.arducamFour;
	}
}
