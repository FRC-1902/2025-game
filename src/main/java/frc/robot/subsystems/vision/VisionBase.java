package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

public interface VisionBase {
  @AutoLog
  public static class VisionInputs {
    public double timestamp;

    // If a camera can see a target
    public boolean arducamOne = false;
    public boolean arducamTwo = false;
    public boolean arducamThree = false;

    // Camera latencies in milliseconds
    public double arducamOneLatencyMS = 0.0;
    public double arducamTwoLatencyMS = 0.0;
    public double arducamThreeLatencyMS = 0.0;

    // Gets the best target ID from somewhere
    public double arducamOneBestTargetID = -1.0;
    public double arducamTwoBestTargetID = -1.0;
    public double arducamThreeBestTargetID = -1.0;

    public Pose3d[] visibleTagPoses = new Pose3d[0];
  }

  public void updateInputs(VisionInputs inputs);

  public void updatePoseEstimation(Pose2d currentPose);

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose();
}