package frc.robot.commands;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignOnTheFly extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private PathPlannerPath m_path;
    private PathConstraints constraints;

    public AlignOnTheFly(String destination, CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        if (destination.equals("collector")) {
            createCollectorLookup();
        } else if (destination.equals("rightReef")) {
            createRightReefLookup();
        } else if (destination.equals("leftReef")) {
            createLeftReefLookup();
        }
        addRequirements(drivetrain);
    }

    private ArrayList<Pose2d> poseLookupList;

    private void createCollectorLookup() {
        poseLookupList = new ArrayList<Pose2d>();
        // blue collectors
        poseLookupList.add(new Pose2d(1.12, 1.05, new Rotation2d(Units.degreesToRadians(234.72))));
        poseLookupList.add(new Pose2d(1.13, 6.46, new Rotation2d(Units.degreesToRadians(125.83))));
        // red collectors
        poseLookupList.add(new Pose2d(15.48, 6.55, new Rotation2d(Units.degreesToRadians(53.18))));
        poseLookupList.add(new Pose2d(15.48, 1.00, new Rotation2d(Units.degreesToRadians(306.42))));
    }

    private void createRightReefLookup() {
        poseLookupList = new ArrayList<Pose2d>();
        // blue right reefs
        poseLookupList.add(new Pose2d(3.84, 2.14, new Rotation2d(Units.degreesToRadians(61.29))));
        poseLookupList.add(new Pose2d(5.42, 2.53, new Rotation2d(Units.degreesToRadians(118.28))));
        poseLookupList.add(new Pose2d(5.88, 4.12, new Rotation2d(Units.degreesToRadians(181.65))));
        poseLookupList.add(new Pose2d(4.78, 5.32, new Rotation2d(Units.degreesToRadians(238.28))));
        poseLookupList.add(new Pose2d(3.06, 4.90, new Rotation2d(Units.degreesToRadians(301.07))));
        poseLookupList.add(new Pose2d(2.67, 3.31, new Rotation2d(Units.degreesToRadians(358.94))));
        // red right reefs
        poseLookupList.add(new Pose2d(10.77, 3.36, new Rotation2d(Units.degreesToRadians(0.88))));
        poseLookupList.add(new Pose2d(11.91, 2.22, new Rotation2d(Units.degreesToRadians(60.15))));
        poseLookupList.add(new Pose2d(13.48, 2.58, new Rotation2d(Units.degreesToRadians(121.71))));
        poseLookupList.add(new Pose2d(13.95, 4.09, new Rotation2d(Units.degreesToRadians(179.51))));
        poseLookupList.add(new Pose2d(12.83, 5.31, new Rotation2d(Units.degreesToRadians(238.33))));
        poseLookupList.add(new Pose2d(11.23, 4.92, new Rotation2d(Units.degreesToRadians(300.18))));
    }

    private void createLeftReefLookup() {
        poseLookupList = new ArrayList<Pose2d>();
        // blue left reefs
        poseLookupList.add(new Pose2d(3.15, 2.55, new Rotation2d(Units.degreesToRadians(61.41))));
        poseLookupList.add(new Pose2d(4.73, 2.14, new Rotation2d(Units.degreesToRadians(120.50))));
        poseLookupList.add(new Pose2d(5.89, 3.35, new Rotation2d(Units.degreesToRadians(178.90))));
        poseLookupList.add(new Pose2d(5.46, 4.93, new Rotation2d(Units.degreesToRadians(235.16))));
        poseLookupList.add(new Pose2d(3.80, 5.30, new Rotation2d(Units.degreesToRadians(299.47))));
        poseLookupList.add(new Pose2d(2.66, 4.16, new Rotation2d(Units.degreesToRadians(359.83))));
        // red left reefs
        poseLookupList.add(new Pose2d(11.26, 2.58, new Rotation2d(Units.degreesToRadians(61.56))));
        poseLookupList.add(new Pose2d(12.79, 2.21, new Rotation2d(Units.degreesToRadians(120.97))));
        poseLookupList.add(new Pose2d(13.96, 3.39, new Rotation2d(Units.degreesToRadians(181.12))));
        poseLookupList.add(new Pose2d(13.57, 4.88, new Rotation2d(Units.degreesToRadians(238.06))));
        poseLookupList.add(new Pose2d(11.92, 5.32, new Rotation2d(Units.degreesToRadians(302.34))));
        poseLookupList.add(new Pose2d(10.76, 4.13, new Rotation2d(Units.degreesToRadians(0.54))));
    }

    @Override
    public void initialize() {
        if (poseLookupList != null && poseLookupList.size() > 0) {
            Pose2d currentPose = m_drivetrain.getState().Pose;
            ArrayList<Double> distance = new ArrayList<>();
            for (int i = 0; i < poseLookupList.size(); i++) {
                Pose2d pose = poseLookupList.get(i);
                //System.out.println(pose);
                double newDistance = Math.sqrt(Math.pow(pose.getX() - currentPose.getX(), 2)
                        + Math.pow(pose.getY() - currentPose.getY(), 2));
                distance.add(newDistance);
                //System.out.println("newDistance: " + newDistance);
            }
            Optional<Double> minDistance = distance.stream().min(Comparator.naturalOrder());
            int indexOfMin = -1;
            if (minDistance.isPresent()) {
                indexOfMin = distance.indexOf(minDistance.get());
                //System.out.println("indexOfMin: " + indexOfMin);
            }
            //System.out.println("AlignOnTheFly currentPose: " + currentPose);
            Pose2d m_destinationPose = poseLookupList.get(indexOfMin);
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                    currentPose, m_destinationPose);

            constraints = new PathConstraints(0.5, 0.5, 2 * Math.PI, 4 * Math.PI); // The constraints for
                                                                                   // this
                                                                                   // path.
            // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); //
            // You can also use unlimited constraints, only limited by motor torque and
            // nominal battery voltage

            // Create the path using the waypoints created above
            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null, // The ideal starting state, this is only relevant for pre-planned paths, so can
                          // be null for on-the-fly paths.
                    new GoalEndState(0.0, m_destinationPose.getRotation()) // Goal end state. You can set a holonomic
                                                                           // rotation
                                                                           // here. If
                                                                           // using a differential drivetrain, the
                                                                           // rotation
                                                                           // will have no
                                                                           // effect.
            );
            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;

            m_path = path;
        }
    }

    @Override
    public void execute() {
        if (m_path != null) {
            System.out.println("Pose2d currentPose = " + m_drivetrain.getState().Pose);
            AutoBuilder.followPath(m_path).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
