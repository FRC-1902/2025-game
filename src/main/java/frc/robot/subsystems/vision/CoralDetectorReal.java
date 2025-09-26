package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Vision;
import edu.wpi.first.units.measure.Distance;
import java.lang.Double;

@Logged
public class CoralDetectorReal implements CoralDetector {

    private final InterpolatingDoubleTreeMap distanceMap;
    private final List<PhotonTrackedTarget> sortedDetections;
    private final List<PhotonTrackedTarget> algaeDetections;
    private final Comparator<PhotonTrackedTarget> detectionTYComparator;
    private final Comparator<PhotonTrackedTarget> detectionTXYComparator;

    // How close to the robot a detected coral has to be to be considered "close" (i.e. intakeable)
    private final static double CLOSE_CORAL_DISTANCE = 0.6;
    private final static double CLOSE_CORAL_TX = 10;
    // How close two detected coral have to be to each other to be considered the same/close enough 
    // to allow switching without a timeout
    private final static double SIMILAR_CORAL_THRESHOLD = 0.75;

    private static final double ALGAE_AVOID_THRESHOLD_DEGREES = 2; // 4.5;

    private double lastDetectionTime = 0;
    private double lastDetectionDistance = 0;
    private double lastDetectionTX = 0;
    private double lastDetectionWidth = 0;
    private double lastDetectionHeight = 0;
    private double lastDetectionRatio = 0;
    private Pose2d lastDetection = null;

    // Additional flags for viewing in logs
    private boolean newCoralValue = false;
    private boolean returningCloseDetection = false;
    private boolean rejectionAlgae = false;
    private boolean rejectionOutsideField = false;
    



    public CoralDetectorReal() {
        distanceMap = new InterpolatingDoubleTreeMap();
        addDistance(-22.2, 18.5);
        addDistance(-14.79, 24);
        addDistance(-6.05, 32);
        addDistance(5.44, 48);
        addDistance(14.89, 72);
        addDistance(20.82, 100);
        addDistance(24.7, 132);

        sortedDetections = new ArrayList<>();
        algaeDetections = new ArrayList<>();
        detectionTYComparator = (a, b) -> Double.compare(a.getYaw(), b.getPitch());
        detectionTXYComparator = (a, b) -> Double.compare(tXYCombined(a), tXYCombined(b));
    }

    private void addDistance(double ty, double inches) {
        distanceMap.put(ty, Inches.of(inches).in(Meters));
    }

    @Override
    public Pose2d getCoralPose(Pose2d robotPose, PhotonTrackedTarget[] detections) {
        newCoralValue = false;
        returningCloseDetection = false;
        rejectionAlgae = false;
        rejectionOutsideField = false;
        sortedDetections.clear();
        algaeDetections.clear();
        
        Pose2d recentLastDetection = getRecentLastDetection();
        if (robotPose == null || detections == null || detections.length == 0) {
            return recentLastDetection;
        }

        boolean auto = RobotState.isAutonomous();

        // If the last detected coral was very close to the robot, wait a bit in case
        // we're trying to intake it
        if (recentLastDetection != null && auto && lastDetectionClose()) {
            returningCloseDetection = true;
            return recentLastDetection;
        }

        for (PhotonTrackedTarget detection : detections) {
            if (detection.getFiducialId() == 1) {
                sortedDetections.add(detection);
            } else {
                algaeDetections.add(detection);
            }
        }
        if (auto) {
            sortedDetections.sort(detectionTYComparator);
        } else {
            sortedDetections.sort(detectionTXYComparator);
        }

        Distance translationX = Vision.CAMERA_THREE_POS.getMeasureX(); 
        Distance translationY = Vision.CAMERA_THREE_POS.getMeasureY();
        Rotation3d rotation = Vision.CAMERA_THREE_POS.getRotation();  
        Rotation2d adjustedRotation = Rotation2d.fromRadians(rotation.getAngle());

        Transform2d cameraTransform = new Transform2d(translationX, translationY, adjustedRotation);
        Pose2d basePose = robotPose.transformBy(cameraTransform);

        for (PhotonTrackedTarget detection : sortedDetections) {
            double degrees = Vision.CAMERA_OBJECT.HORIZONTAL_FOV.getDegrees();

            if (auto) {
                // Skip any coral that are close to an algae on the X axis - these are likely lollipops
                for (PhotonTrackedTarget algae : algaeDetections) {
                    if (Math.abs(degrees - algae.getPitch()) < ALGAE_AVOID_THRESHOLD_DEGREES) {// && detection.tync < algae.tync) {
                        rejectionAlgae = true;
                        break;
                    }
                }
                if (rejectionAlgae) continue;
            }

            double distanceMeters = distanceMap.get(detection.getPitch());
            double radians = Units.degreesToRadians(degrees);
            double yComponent = distanceMeters * Math.tan(radians);
            Transform2d coralTransform = new Transform2d(distanceMeters, -yComponent, Rotation2d.kZero);
            Pose2d coralPose = basePose.transformBy(coralTransform);

            if (!CoralDetector.isValid(coralPose)) {
                rejectionOutsideField = true;
                continue;
            }

            double robotDist = coralPose.getTranslation().getDistance(robotPose.getTranslation());

            lastDetection = coralPose;
            lastDetectionTime = Timer.getFPGATimestamp();
            lastDetectionDistance = robotDist;
            lastDetectionTX = detection.getYaw();
            lastDetectionWidth = width(detection);
            lastDetectionHeight = height(detection);
            lastDetectionRatio = lastDetectionWidth / lastDetectionHeight;
            newCoralValue = true;
            return coralPose;
        }

        // If we didn't find any coral, return the last detection if it was very recent
        return recentLastDetection;
    }

    @Override
    public void reset() {
        lastDetection = null;
        lastDetectionDistance = 0;
        lastDetectionTime = 0;
    }

    private Pose2d getRecentLastDetection() {
        boolean lastClose = lastDetectionClose();
        boolean auto = RobotState.isAutonomous();

        double timeoutSeconds = lastClose ? 3 : 0.5;
        if (auto && lastClose) timeoutSeconds = 1;
        if (Timer.getFPGATimestamp() - lastDetectionTime < timeoutSeconds) {
            return lastDetection;
        }
        return null;
    }

    public boolean lastDetectionClose() {
        if (lastDetection == null) return false;

        return lastDetectionDistance < CLOSE_CORAL_DISTANCE;
    }

    private double tXYCombined(PhotonTrackedTarget detection) {
        return detection.getPitch() + Math.abs(detection.getYaw() * 0.75);
    }

    private double width(PhotonTrackedTarget detection) {
        List <TargetCorner> corners = detection.getDetectedCorners(); 
        double minX = corners.stream().mapToDouble(c -> c.x).min().orElse(0);
        double maxX = corners.stream().mapToDouble(c -> c.x).max().orElse(0);
        return Math.abs(maxX - minX); 
    }

    private double height(PhotonTrackedTarget detection) {
        List <TargetCorner> corners = detection.getDetectedCorners();
        double minY = corners.stream().mapToDouble(c -> c.y).min().orElse(0);
        double maxY = corners.stream().mapToDouble(c -> c.y).max().orElse(0);
        return Math.abs(maxY - minY);
    }
}
