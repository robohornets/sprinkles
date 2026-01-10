package frc.robot.subsystems.photonvision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionSubsystem extends SubsystemBase {

    RobotContainer robotContainer = new RobotContainer();
    
    // Instantiate PhotonCameras for each of the four cameras
    private final PhotonCamera cameraFrontLeft  = new PhotonCamera("FrontLeft");
    private final PhotonCamera cameraFrontRight = new PhotonCamera("FrontRight");
    private final PhotonCamera cameraBackLeft   = new PhotonCamera("BackLeft");
    private final PhotonCamera cameraBackRight  = new PhotonCamera("BackRight");

    // Define the transform from the robot's reference point to each camera
    private final Transform3d robotToCamFrontLeft  = new Transform3d(
        new Translation3d(0.3, 0.2, 0.5), 
        new Rotation3d(0.0, Math.toRadians(15), 0.0)
    );
    private final Transform3d robotToCamFrontRight = new Transform3d(
        new Translation3d(0.3, 0.2, 0.5), 
        new Rotation3d(0.0, Math.toRadians(15), 0.0)
    );
    private final Transform3d robotToCamBackLeft   = new Transform3d(
        new Translation3d(0.3, 0.2, 0.5), 
        new Rotation3d(0.0, Math.toRadians(15), 0.0)
    );
    private final Transform3d robotToCamBackRight  = new Transform3d(
        new Translation3d(0.3, 0.2, 0.5), 
        new Rotation3d(0.0, Math.toRadians(15), 0.0)
    );

    // Load the AprilTagFieldLayout for the 2025 game
    private final AprilTagFieldLayout fieldLayout = loadAprilTagFieldLayout();
    

    // Create PhotonPoseEstimators for each camera
    private final PhotonPoseEstimator estimatorFrontLeft  = 
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamFrontLeft);
    private final PhotonPoseEstimator estimatorFrontRight = 
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamFrontRight);
    private final PhotonPoseEstimator estimatorBackLeft   = 
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamBackLeft);
    private final PhotonPoseEstimator estimatorBackRight  = 
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamBackRight);

    // Hold the latest estimated robot pose
    public Pose2d latestEstimatedPose = new Pose2d();

    public VisionSubsystem() {
        // Additional initialization if needed
    }

    @Override
    public void periodic() {
        // Update the pose estimate periodically
        updatePose();
    }

    /**
     * Retrieves the current odometry-based pose.
     * Replace this with your drivetrain's odometry retrieval.
     */
    private Pose2d getOdometryPose() {
        

        return new Pose2d();
    }

    /**
     * Updates the robot's pose estimate using PhotonVision data from all cameras.
     */
    public void updatePose() {
        // Get the current odometry pose to provide a reference
        Pose2d currentOdometryPose = getOdometryPose();

        
    }


    /**
     * Loads the AprilTagFieldLayout for the 2025 game.
     */
    private AprilTagFieldLayout loadAprilTagFieldLayout() {
        try {
            // Load the field layout for the 2025 game
            return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }
}
