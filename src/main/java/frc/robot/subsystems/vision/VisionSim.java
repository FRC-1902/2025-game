package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Creates a simulated vision system from the vision base
 */
public class VisionSim implements VisionBase {
    private final VisionSystemSim visionSim;
    private final Map<Constants.Vision.Camera, PhotonCameraSim> cameraSims = new EnumMap<>(Constants.Vision.Camera.class);
    private final Map<Constants.Vision.Camera, PhotonPipelineResult> currentResults = new EnumMap<>(Constants.Vision.Camera.class);
    private Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();
    private final AprilTagFieldLayout fieldLayout;

    public VisionSim() {
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        visionSim = new VisionSystemSim("Vision");
        visionSim.addAprilTags(fieldLayout);

        // Set camera properties
        SimCameraProperties properties = new SimCameraProperties();
        properties.setCalibration(960, 720, new Rotation2d(Math.toRadians(100)));
        properties.setCalibError(0.25, 0.08);
        properties.setFPS(30);
        properties.setAvgLatencyMs(35);
        properties.setLatencyStdDevMs(5);

        // Add cameras
        for (Constants.Vision.Camera cam : Constants.Vision.Camera.values()) {
            PhotonCamera camera = new PhotonCamera(cam.name);
            PhotonCameraSim cameraSim = new PhotonCameraSim(camera, properties);
            cameraSim.enableDrawWireframe(true);
            cameraSims.put(cam, cameraSim);
            currentResults.put(cam, new PhotonPipelineResult()); // Empty initial result

            Transform3d robotToCam = new Transform3d(cam.translation, cam.rotation);
            visionSim.addCamera(cameraSim, robotToCam);
        }
    }

    /**
     * Updates vision inputs with the latest target info from each camera
     */
    @Override
    public void updateInputs(VisionInputs inputs) {
        List<Pose3d> visibleTagPoses = new ArrayList<>();

        // Update all camera results first
        for (Map.Entry<Constants.Vision.Camera, PhotonCameraSim> entry : cameraSims.entrySet()) {
            Constants.Vision.Camera cam = entry.getKey();
            PhotonCamera camera = entry.getValue().getCamera();

            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            if (!results.isEmpty()) {
                // Get the most recent result
                results.sort((a, b) -> Double.compare(b.getTimestampSeconds(), a.getTimestampSeconds()));
                currentResults.put(cam, results.get(0));
            }
        }

        // Process the results for each camera
        for (Map.Entry<Constants.Vision.Camera, PhotonCameraSim> entry : cameraSims.entrySet()) {
            Constants.Vision.Camera cam = entry.getKey();
            PhotonPipelineResult result = currentResults.get(cam);

            // Collect visible tag poses
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
                    tagPose.ifPresent(
                        pose -> {
                            if (!visibleTagPoses.contains(pose)) {
                                visibleTagPoses.add(pose);
                            }
                        }
                    );
                }

                // TODO: Change/Switch to actual cameras
                switch (cam) {
                    case ArducamOne:
                        inputs.arducamOne = true;
                        inputs.arducamOneBestTargetID = result.getBestTarget().getFiducialId();
                        break;
                    case ArducamTwo:
                        inputs.arducamTwo = true;
                        inputs.arducamTwoBestTargetID = result.getBestTarget().getFiducialId();
                        break;
                    case ArducamThree:
                        inputs.arducamThree = true;
                        inputs.arducamThreeBestTargetID = result.getBestTarget().getFiducialId();
                        break;
                }
            }
        }

        // Update inputs with visible tag poses
        inputs.visibleTagPoses = visibleTagPoses.toArray(new Pose3d[0]);
        Logger.recordOutput("Vision/TagPoses", inputs.visibleTagPoses);

        // Record estimated pose output
        SmartDashboard.putData("Vision/CurrentEstimatedPose", visionSim.getDebugField());
    }

    /**
     * Updates the vision with the current robot pose
     */
    @Override
    public void updatePoseEstimation(Pose2d currentPose) {
        visionSim.update(currentPose);
    }

    /**
     * Gets the estimated global robot pose from the vision 
     */
    @Override
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return lastEstimatedPose;
    }
}
