package frc.robot.subsystems.photonvision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.photonvision.VisionConstants;

public class Vision {
    private final PhotonCamera camera;
    private final PhotonCamera cameraR;
    private final PhotonCamera cameraF;
    private final PhotonCamera cameraB;
    private final PhotonPoseEstimator photonEstimator;
    private final PhotonPoseEstimator photonEstimatorR;
    private final PhotonPoseEstimator photonEstimatorF;
    private final PhotonPoseEstimator photonEstimatorB;
    private Matrix<N3, N1> curStdDevs;
    

    public Vision(){
        camera = new PhotonCamera(VisionConstants.Vision.kCameraName);
        cameraR = new PhotonCamera(VisionConstants.Vision.kCameraNameR);
        cameraF = new PhotonCamera(VisionConstants.Vision.kCameraNameF);
        cameraB = new PhotonCamera(VisionConstants.Vision.kCameraNameB);

        photonEstimator =
                new PhotonPoseEstimator(VisionConstants.Vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.Vision.kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorR =
                new PhotonPoseEstimator(VisionConstants.Vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.Vision.kRobotToCamR);
        photonEstimatorR.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorF =
                new PhotonPoseEstimator(VisionConstants.Vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.Vision.kRobotToCamF);
        photonEstimatorF.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorB =
                new PhotonPoseEstimator(VisionConstants.Vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.Vision.kRobotToCamB);
        photonEstimatorB.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

 public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    // Collect results from all cameras
    List<PhotonTrackedTarget> allTargets = new ArrayList<>();
    Map<PhotonTrackedTarget, Double> targetDistanceMap = new HashMap<>();

    // Iterate over each camera and collect targets
    for (PhotonCamera camera : List.of(this.camera, this.cameraR, this.cameraF, this.cameraB)) {
        var latestResult = camera.getLatestResult();
        if (latestResult.hasTargets()) {
            for (var target : latestResult.getTargets()) {
                allTargets.add(target);
                targetDistanceMap.put(target, target.getBestCameraToTarget().getTranslation().getNorm());
            }
        }
    }

    // Select the best target based on number of targets and minimum distance
    Optional<PhotonTrackedTarget> bestTarget = allTargets.stream()
        .min(Comparator.comparingDouble(targetDistanceMap::get));

    // If a best target is found, get the estimated pose for that target
    if (bestTarget.isPresent()) {
        PhotonTrackedTarget target = bestTarget.get();
        for (PhotonCamera camera : List.of(this.camera, this.cameraR, this.cameraF, this.cameraB)) {
            var result = camera.getLatestResult();
            if (result.getTargets().contains(target)) {
                return photonEstimator.update(result);
            }
        }
    }

    return Optional.empty();
}

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.Vision.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.Vision.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.Vision.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.Vision.kSingleTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
}
