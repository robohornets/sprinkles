package frc.robot.subsystems.photonvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
    public static class Vision {
        public static final String kCameraName = "Camera Face Left";
        public static final String kCameraNameR = "Camera Face Right";
        public static final String kCameraNameB = "Camera Face Back";
        public static final String kCameraNameF = "Camera Face Front";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.3, 0.2, 0.5), new Rotation3d(0.0, Math.toRadians(15), 0.0));
        public static final Transform3d robotToCamFrontRight = new Transform3d(
            new Translation3d(0.3, 0.2, 0.5), 
            new Rotation3d(0.0, Math.toRadians(15), 0.0)
        );
        public static final Transform3d robotToCamBackLeft   = new Transform3d(
            new Translation3d(0.3, 0.2, 0.5), 
            new Rotation3d(0.0, Math.toRadians(15), 0.0)
        );
        public static final Transform3d robotToCamBackRight  = new Transform3d(
            new Translation3d(0.3, 0.2, 0.5), 
            new Rotation3d(0.0, Math.toRadians(15), 0.0)
        );

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}
