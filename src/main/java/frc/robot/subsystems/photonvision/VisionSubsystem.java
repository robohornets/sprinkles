// VisionSubsystem.java – handles AprilTag data from PhotonVision
package frc.robot.subsystems.photonvision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    // PhotonVision camera and pose estimator
    private final PhotonCamera camFaceFront;
    private final PhotonCamera camFaceBack;
    private final PhotonCamera camFaceRight;
    private final PhotonCamera camFaceLeft;
    // (Add additional cameras as needed: left/right, etc.)
    private final PhotonPoseEstimator poseEstimatorFront;
    private final PhotonPoseEstimator poseEstimatorBack;
    private final PhotonPoseEstimator poseEstimatorRight;
    private final PhotonPoseEstimator poseEstimatorLeft;
    
    // Field layout for AprilTags (use correct game field layout)
    private final AprilTagFieldLayout aprilTagLayout;
    
    public VisionSubsystem() {
        aprilTagLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
        
        // Instantiate PhotonCamera for each camera (names must match PhotonVision config)
        camFaceFront = new PhotonCamera("Camera Face Front");
        camFaceBack  = new PhotonCamera("Camera Face Back");
        camFaceRight = new PhotonCamera("Camera Face Right");
        camFaceLeft = new PhotonCamera("Camera Face Left");
        
        // Define the transform from robot center to each camera.
        Transform3d robotTocamFaceFront = new Transform3d(
                new Translation3d(0.35, -0.22, 0.9),
                new Rotation3d(0.0, 0.0, 0.0));
                
        // camFaceBack: 0.5m to rear, facing backward (rotate 180 degrees around yaw)
        Transform3d robotTocamFaceBack = new Transform3d(
                new Translation3d(-0.35, 0.27, 0.9),
                new Rotation3d(0.0, 0.0, Math.toRadians(180)));

        Transform3d robotTocamFaceRight = new Transform3d(
            new Translation3d(-0.23, -0.35, 0.9),
            new Rotation3d(0.0, 0.0, Math.toRadians(270))
        );

        Transform3d robotTocamFaceLeft = new Transform3d(
            new Translation3d(-0.2, 0.35, 0.9),
            new Rotation3d(0.0, 0.0, Math.toRadians(90))
        );
        
        // Create PhotonPoseEstimator for each camera, using a strategy to combine tag data
        poseEstimatorFront = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotTocamFaceFront);

        poseEstimatorBack = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotTocamFaceBack);

        poseEstimatorRight = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotTocamFaceRight);

        poseEstimatorLeft = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotTocamFaceLeft);

        // You can use MULTI_TAG_PNP_ON_COPROCESSOR for highest accuracy if PhotonVision is set up for it.
        // LOWEST_AMBIGUITY will choose the best pose if tags have ambiguous solutions.
    }
    
    /**
     * Estimate the robot's field pose from camera data.
     * @param prevEstimatedRobotPose The current pose estimate (from odometry) to use as a reference.
     * @return An Optional containing the estimated global pose and timestamp if vision data is available.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(edu.wpi.first.math.geometry.Pose2d prevEstimatedRobotPose) {
        // Debug: Print the previous estimated pose
        System.out.println("Previous Estimated Pose: " + prevEstimatedRobotPose);
    
        Optional<EstimatedRobotPose> frontResult = Optional.empty();
        if (camFaceFront.getLatestResult().hasTargets()) {
            System.out.println("[Front] Detected " + camFaceFront.getLatestResult().getTargets().size() + " targets.");
            
            poseEstimatorFront.setReferencePose(prevEstimatedRobotPose);
            PhotonPipelineResult frontResultData = camFaceFront.getLatestResult();
            frontResult = poseEstimatorFront.update(frontResultData);
    
            if (frontResult.isPresent()) {
                System.out.println("[Front] Estimated Pose: " + frontResult.get().estimatedPose);
            } else {
                System.out.println("[Front] No pose estimate.");
            }
        } else {
            System.out.println("[Front] No targets detected.");
        }
    
        Optional<EstimatedRobotPose> backResult = Optional.empty();
        if (camFaceBack.getLatestResult().hasTargets()) {
            System.out.println("[Back] Detected " + camFaceBack.getLatestResult().getTargets().size() + " targets.");
    
            poseEstimatorBack.setReferencePose(prevEstimatedRobotPose);
            PhotonPipelineResult backResultData = camFaceBack.getLatestResult();
            backResult = poseEstimatorBack.update(backResultData);
    
            if (backResult.isPresent()) {
                System.out.println("[Back] Estimated Pose: " + backResult.get().estimatedPose);
            } else {
                System.out.println("[Back] No pose estimate.");
            }
        } else {
            System.out.println("[Back] No targets detected.");
        }
    
        Optional<EstimatedRobotPose> rightResult = Optional.empty();
        if (camFaceRight.getLatestResult().hasTargets()) {
            System.out.println("[Right] Detected " + camFaceRight.getLatestResult().getTargets().size() + " targets.");
    
            poseEstimatorRight.setReferencePose(prevEstimatedRobotPose);
            PhotonPipelineResult rightResultData = camFaceRight.getLatestResult();
            rightResult = poseEstimatorRight.update(rightResultData);
    
            if (rightResult.isPresent()) {
                System.out.println("[Right] Estimated Pose: " + rightResult.get().estimatedPose);
            } else {
                System.out.println("[Right] No pose estimate.");
            }
        } else {
            System.out.println("[Right] No targets detected.");
        }
    
        Optional<EstimatedRobotPose> leftResult = Optional.empty();
        if (camFaceLeft.getLatestResult().hasTargets()) {
            System.out.println("[Left] Detected " + camFaceLeft.getLatestResult().getTargets().size() + " targets.");
    
            poseEstimatorLeft.setReferencePose(prevEstimatedRobotPose);
            PhotonPipelineResult leftResultData = camFaceLeft.getLatestResult();
            leftResult = poseEstimatorLeft.update(leftResultData);
    
            if (leftResult.isPresent()) {
                System.out.println("[Left] Estimated Pose: " + leftResult.get().estimatedPose);
            } else {
                System.out.println("[Left] No pose estimate.");
            }
        } else {
            System.out.println("[Left] No targets detected.");
        }
    
        // Choose the best result among all cameras based on pose ambiguity
        Optional<EstimatedRobotPose> bestResult = Optional.empty();
        double bestAmbiguity = Double.MAX_VALUE;
    
        // Check Front Camera
        if (frontResult.isPresent()) {
            double frontAmb = camFaceFront.getLatestResult().getBestTarget().getPoseAmbiguity();
            System.out.println("[Front] Pose Ambiguity: " + frontAmb);
            if (!Double.isNaN(frontAmb) && frontAmb >= 0 && frontAmb < bestAmbiguity) {
                bestAmbiguity = frontAmb;
                bestResult = frontResult;
            }
        }
    
        // Check Back Camera
        if (backResult.isPresent()) {
            double backAmb = camFaceBack.getLatestResult().getBestTarget().getPoseAmbiguity();
            System.out.println("[Back] Pose Ambiguity: " + backAmb);
            if (!Double.isNaN(backAmb) && backAmb >= 0 && backAmb < bestAmbiguity) {
                bestAmbiguity = backAmb;
                bestResult = backResult;
            }
        }
    
        // Check Right Camera
        if (rightResult.isPresent()) {
            double rightAmb = camFaceRight.getLatestResult().getBestTarget().getPoseAmbiguity();
            System.out.println("[Right] Pose Ambiguity: " + rightAmb);
            if (!Double.isNaN(rightAmb) && rightAmb >= 0 && rightAmb < bestAmbiguity) {
                bestAmbiguity = rightAmb;
                bestResult = rightResult;
            }
        }
    
        // Check Left Camera
        if (leftResult.isPresent()) {
            double leftAmb = camFaceLeft.getLatestResult().getBestTarget().getPoseAmbiguity();
            System.out.println("[Left] Pose Ambiguity: " + leftAmb);
            if (!Double.isNaN(leftAmb) && leftAmb >= 0 && leftAmb < bestAmbiguity) {
                bestAmbiguity = leftAmb;
                bestResult = leftResult;
            }
        }
    
        // Debug: Print the final best estimated pose
        if (bestResult.isPresent()) {
            System.out.println("Best Estimated Pose Selected: " + bestResult.get().estimatedPose);
        } else {
            System.out.println("No best estimated pose found.");
        }
    
        return bestResult;
    }
    

    public Pose2d getLatestEstimatedPose(Pose2d fallbackPose) {
        Optional<EstimatedRobotPose> visionPoseOpt = getEstimatedGlobalPose(fallbackPose);
        if (visionPoseOpt.isPresent()) {
            // Convert the 3D pose to a 2D pose (assumes Z is not used)
            return visionPoseOpt.get().estimatedPose.toPose2d();
        }
        return fallbackPose;
    }
}
