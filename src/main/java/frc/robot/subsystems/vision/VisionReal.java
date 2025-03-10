package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// Creates real vision system from vision base
public class VisionReal implements VisionBase {
  private final AprilTagFieldLayout fieldLayout;
  private final Map<Constants.Vision.Camera, PhotonCamera> cameras = new EnumMap<>(Constants.Vision.Camera.class);
  private final Map<Constants.Vision.Camera, PhotonPoseEstimator> poseEstimators = new EnumMap<>(Constants.Vision.Camera.class);
  private Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();

  // Current results for each camera, updated in updateInputs
  private final Map<Constants.Vision.Camera, PhotonPipelineResult> currentResults = new EnumMap<>(Constants.Vision.Camera.class);
  private final Map<Constants.Vision.Camera, Matrix<N3, N1>> currentStdDevs = new EnumMap<>(Constants.Vision.Camera.class);

  /** Constructor initializes all cameras and their pose estimators */
  public VisionReal() {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    for (Constants.Vision.Camera cam : Constants.Vision.Camera.values()) {
      cameras.put(cam, new PhotonCamera(cam.camName));
      currentResults.put(cam, new PhotonPipelineResult()); // Empty initial result
      currentStdDevs.put(cam, cam.singleTagStdDevs);

      Transform3d robotToCam = new Transform3d(cam.translation, cam.rotation);
      PhotonPoseEstimator estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

      estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      poseEstimators.put(cam, estimator);
    }
  }

  /** Updates vision inputs with the latest target information from each camera */
  @Override
  public void updateInputs(VisionInputs inputs) {
    List<Pose3d> visibleTagPoses = new ArrayList<>();

    // Update all camera results first
    for (Map.Entry<Constants.Vision.Camera, PhotonCamera> entry : cameras.entrySet()) {
      Constants.Vision.Camera cam = entry.getKey();
      PhotonCamera camera = entry.getValue();

      List<PhotonPipelineResult> results = camera.getAllUnreadResults();
      if (!results.isEmpty()) {
        // Get the most recent result
        results.sort((a, b) -> Double.compare(b.getTimestampSeconds(), a.getTimestampSeconds()));
        currentResults.put(cam, results.get(0));
      }
    }

    // Process the results for each camera
    for (Map.Entry<Constants.Vision.Camera, PhotonCamera> entry : cameras.entrySet()) {
      Constants.Vision.Camera cam = entry.getKey();
      PhotonPipelineResult result = currentResults.get(cam);

      if (result.hasTargets()) {
          PhotonTrackedTarget bestTarget = result.getBestTarget();

          // Collect poses of all visible tags
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

          // TODO: Add/Change Cameras
        switch (cam) {
          case ArducamOne:
            inputs.arducamOne = true;
            inputs.arducamOneBestTargetID = bestTarget.getFiducialId();
            break;
          case ArducamTwo:
            inputs.arducamTwo = true;
            inputs.arducamTwoBestTargetID = bestTarget.getFiducialId();
            break;
          case ArducamThree:
            inputs.arducamThree = true;
            inputs.arducamThreeBestTargetID = bestTarget.getFiducialId();
            break;
        }
      }
    }

    // Update inputs with visible tag poses
    inputs.visibleTagPoses = visibleTagPoses.toArray(new Pose3d[0]);
  }

  /** Updates robot pose estimation using data from all cameras */
  public void updatePoseEstimation(Pose2d currentPose) {
    Map<Constants.Vision.Camera, EstimatedRobotPose> cameraEstimates = new EnumMap<>(Constants.Vision.Camera.class);

    for (Map.Entry<Constants.Vision.Camera, PhotonPoseEstimator> entry : poseEstimators.entrySet()) {
      Constants.Vision.Camera cam = entry.getKey();
      PhotonPoseEstimator estimator = entry.getValue();
      PhotonPipelineResult result = currentResults.get(cam);

      if (!result.hasTargets()) {
        continue;
      }

      // Filter out unreliable single-tag measurements
      if (result.getTargets().size() == 1) {
        PhotonTrackedTarget target = result.getBestTarget();
        if (target.getPoseAmbiguity() > Constants.Vision.MAXIMUM_AMBIGUITY) {
          Logger.recordOutput(
            "Vision/" + cam.camName + "/RejectedAmbiguity", target.getPoseAmbiguity());
          continue;
        }
      }

      // Set reference pose to current robot pose
      estimator.setReferencePose(currentPose);

      // Update estimator
      Optional<EstimatedRobotPose> poseResult = estimator.update(result);

      // If estimation is successful record output, update estimates, and update stdDevs
      if (poseResult.isPresent()) {
        EstimatedRobotPose estimate = poseResult.get();
        updateEstimationStdDevs(cam, poseResult, result.getTargets());
        cameraEstimates.put(cam, estimate);

        Logger.recordOutput("Vision/" + cam.camName + "/EstimatedPose", estimate.estimatedPose);
        Logger.recordOutput("Vision/" + cam.camName + "/TimestampSeconds", estimate.timestampSeconds);
        Logger.recordOutput("Vision/" + cam.camName + "/TagCount", estimate.targetsUsed.size());
      }
    }

    if (!cameraEstimates.isEmpty()) {
      EstimatedRobotPose combinedPose = combineEstimates(cameraEstimates);
      lastEstimatedPose = Optional.of(combinedPose);
      Logger.recordOutput("Vision/CombinedEstimatedPose", combinedPose.estimatedPose);
    }
  }

  private EstimatedRobotPose combineEstimates(
    Map<Constants.Vision.Camera, EstimatedRobotPose> estimates) {
    double weightedX = 0;
    double weightedY = 0;
    double weightedRot = 0;
    double totalWeightX = 0;
    double totalWeightY = 0;
    double totalWeightRot = 0;

    EstimatedRobotPose firstEstimate = estimates.values().iterator().next();

    for (Map.Entry<Constants.Vision.Camera, EstimatedRobotPose> entry : estimates.entrySet()) {
      Constants.Vision.Camera cam = entry.getKey();
      EstimatedRobotPose estimate = entry.getValue();
      Matrix<N3, N1> stdDevs = currentStdDevs.get(cam);

      double weightX = 1.0 / (stdDevs.get(0, 0) * stdDevs.get(0, 0));
      double weightY = 1.0 / (stdDevs.get(1, 0) * stdDevs.get(1, 0));
      double weightRot = 1.0 / (stdDevs.get(2, 0) * stdDevs.get(2, 0));

      Pose3d pose = estimate.estimatedPose;
      weightedX += pose.getX() * weightX;
      weightedY += pose.getY() * weightY;
      weightedRot += pose.getRotation().getZ() * weightRot;

      totalWeightX += weightX;
      totalWeightY += weightY;
      totalWeightRot += weightRot;
    }

    double finalX = weightedX / totalWeightX;
    double finalY = weightedY / totalWeightY;
    double finalRot = weightedRot / totalWeightRot;

    Pose3d combinedPose =
      new Pose3d(new Translation3d(finalX, finalY, 0), new Rotation3d(0, 0, finalRot));

    return new EstimatedRobotPose(
      combinedPose,
      firstEstimate.timestampSeconds,
      firstEstimate.targetsUsed,
      firstEstimate.strategy);
  }

  @Override
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return lastEstimatedPose;
  }

  private void updateEstimationStdDevs(
    Constants.Vision.Camera camera,
    Optional<EstimatedRobotPose> estimatedPose,
    List<PhotonTrackedTarget> targets) {

    if (estimatedPose.isEmpty()) {
      currentStdDevs.put(camera, camera.singleTagStdDevs);
      return;
    }

    Matrix<N3, N1> estStdDevs = camera.singleTagStdDevs;
    int numTags = 0;
    double avgDist = 0;

    for (PhotonTrackedTarget target : targets) {
      Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) {
        continue;
      }
      numTags++;
      avgDist += tagPose
        .get()
        .toPose2d()
        .getTranslation()
        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
    }

    if (numTags == 0) {
      currentStdDevs.put(camera, camera.singleTagStdDevs);
      return;
    }

    avgDist /= numTags;

    if (numTags > 1) {
      estStdDevs = camera.multiTagStdDevs;
    }

    if (numTags == 1 && avgDist > 4) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    }

    currentStdDevs.put(camera, estStdDevs);
  }
}
