package frc.robot.subsystems.photonvision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Vision {

    // Load the field layout
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Cameras and their transforms
    private final PhotonCamera frontCam = new PhotonCamera("FrontCamera");
    private final PhotonCamera backCam = new PhotonCamera("BackCamera");
    private final PhotonCamera leftCam = new PhotonCamera("LeftCamera");
    private final PhotonCamera rightCam = new PhotonCamera("RightCamera");

    private final Transform3d frontCamTransform = new Transform3d(
        new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)); // Front
    private final Transform3d backCamTransform = new Transform3d(
        new Translation3d(-0.5, 0.0, 0.5), new Rotation3d(0, Math.PI, 0)); // Back
    private final Transform3d leftCamTransform = new Transform3d(
        new Translation3d(0.0, -0.5, 0.5), new Rotation3d(0, Math.PI / 2, 0)); // Left
    private final Transform3d rightCamTransform = new Transform3d(
        new Translation3d(0.0, 0.5, 0.5), new Rotation3d(0, -Math.PI / 2, 0)); // Right

    // Pose estimators for each camera
    private final PhotonPoseEstimator frontEstimator;
    private final PhotonPoseEstimator backEstimator;
    private final PhotonPoseEstimator leftEstimator;
    private final PhotonPoseEstimator rightEstimator;

    public Vision() {
        // Initialize PhotonPoseEstimators
        frontEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamTransform);
        backEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamTransform);
        leftEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCamTransform);
        rightEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCamTransform);
    }

    /**
     * Get the combined robot pose from all cameras.
     * @return An Optional containing the combined Pose3d, or empty if no valid pose is available.
     */
    public Optional<Pose3d> getRobotPose() {
        List<Optional<Pose3d>> poseEstimates = new ArrayList<>();

        // Get latest results from each camera and update their estimators
        PhotonPipelineResult frontResult = frontCam.getLatestResult();
        PhotonPipelineResult backResult = backCam.getLatestResult();
        PhotonPipelineResult leftResult = leftCam.getLatestResult();
        PhotonPipelineResult rightResult = rightCam.getLatestResult();

        // Update pose estimators and collect valid poses
        if (frontResult.hasTargets()) {
            poseEstimates.add(frontEstimator.update(frontResult).map(p -> p.estimatedPose));
        }
        if (backResult.hasTargets()) {
            poseEstimates.add(backEstimator.update(backResult).map(p -> p.estimatedPose));
        }
        if (leftResult.hasTargets()) {
            poseEstimates.add(leftEstimator.update(leftResult).map(p -> p.estimatedPose));
        }
        if (rightResult.hasTargets()) {
            poseEstimates.add(rightEstimator.update(rightResult).map(p -> p.estimatedPose));
        }

        // Filter out empty poses
        poseEstimates.removeIf(Optional::isEmpty);

        if (poseEstimates.isEmpty()) {
            return Optional.empty();
        }

        // Average the poses
        double x = 0, y = 0, z = 0;
        double rotX = 0, rotY = 0, rotZ = 0;

        for (Optional<Pose3d> optPose : poseEstimates) {
            Pose3d pose = optPose.get();
            x += pose.getX();
            y += pose.getY();
            z += pose.getZ();
            rotX += pose.getRotation().getX();
            rotY += pose.getRotation().getY();
            rotZ += pose.getRotation().getZ();
        }

        int count = poseEstimates.size();
        return Optional.of(new Pose3d(
            x / count, y / count, z / count, 
            new Rotation3d(rotX / count, rotY / count, rotZ / count)
        ));
    }

    /**
     * Periodic method to update and display the robot pose.
     */
    public void periodic() {
        Optional<Pose3d> robotPose = getRobotPose();
        robotPose.ifPresent(pose -> {
            SmartDashboard.putString("Robot Pose", pose.toPose2d().toString());
        });
    }
}