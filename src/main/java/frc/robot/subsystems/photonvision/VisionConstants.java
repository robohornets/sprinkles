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
                new Transform3d(new Translation3d(-0.2, 0.35, 0.9), 
                new Rotation3d(0.0, 0.0, Math.toRadians(90)));
        public static final Transform3d kRobotToCamR = new Transform3d(
            new Translation3d(-0.23, 0.35, 0.9), 
            new Rotation3d(0.0, 0.0, Math.toRadians(270))
        );
        public static final Transform3d kRobotToCamB   = new Transform3d(
            new Translation3d(-0.35, 0.27, 0.9), 
            new Rotation3d(0.0, 0.0, Math.toRadians(180))
        );
        public static final Transform3d kRobotToCamF  = new Transform3d(
            new Translation3d(0.35, -0.22, 0.9), 
            new Rotation3d(0.0, 0.0, Math.toRadians(0))
        );

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}
