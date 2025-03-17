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


public class AlignOnTheFlyClosest extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private PathPlannerPath m_path;
    private PathConstraints constraints;

    public AlignOnTheFlyClosest(Destinations destination, CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        switch (destination) {
            case COLLECTOR -> createCollectorLookup();
            case RIGHT_REEF -> createRightReefLookup();
            case LEFT_REEF -> createLeftReefLookup();
            default -> throw new IllegalArgumentException("Unknown destination: " + destination);
        }
        
        addRequirements(drivetrain);
    }

    private ArrayList<Pose2d> poseLookupList;

    private void createCollectorLookup() {
        poseLookupList = new ArrayList<Pose2d>();
        // blue collectors
        poseLookupList.add(new Pose2d(1.12, 1.05, new Rotation2d(Units.degreesToRadians(234)))); // angle changed
        poseLookupList.add(new Pose2d(1.656, 7.500, new Rotation2d(Units.degreesToRadians(126)))); //changed
        // red collectors
        poseLookupList.add(new Pose2d(15.48, 6.55, new Rotation2d(Units.degreesToRadians(54)))); // angle changed
        poseLookupList.add(new Pose2d(15.48, 1.00, new Rotation2d(Units.degreesToRadians(306)))); // angle changed
    }

    private void createRightReefLookup() {
        poseLookupList = new ArrayList<Pose2d>();
        // blue right reefs
        poseLookupList.add(new Pose2d(4.138, 2.714, new Rotation2d(Units.degreesToRadians(60)))); 
        poseLookupList.add(new Pose2d(5.474, 3.083, new Rotation2d(Units.degreesToRadians(120)))); 
        poseLookupList.add(new Pose2d(5.782, 4.360, new Rotation2d(Units.degreesToRadians(180)))); 
        poseLookupList.add(new Pose2d(4.788, 5.318, new Rotation2d(Units.degreesToRadians(240))));
        poseLookupList.add(new Pose2d(3.532, 4.964, new Rotation2d(Units.degreesToRadians(300)))); 
        poseLookupList.add(new Pose2d(3.201, 3.661, new Rotation2d(Units.degreesToRadians(360))));
        // red right reefs
        poseLookupList.add(new Pose2d(11.799, 3.695, new Rotation2d(Units.degreesToRadians(0)))); 
        poseLookupList.add(new Pose2d(12.728, 2.725, new Rotation2d(Units.degreesToRadians(60)))); 
        poseLookupList.add(new Pose2d(13.997, 3.086, new Rotation2d(Units.degreesToRadians(120)))); 
        poseLookupList.add(new Pose2d(14.338, 4.366, new Rotation2d(Units.degreesToRadians(180)))); 
        poseLookupList.add(new Pose2d(13.430, 5.294, new Rotation2d(Units.degreesToRadians(240)))); 
        poseLookupList.add(new Pose2d(12.119, 4.964, new Rotation2d(Units.degreesToRadians(300))));
    }

    private void createLeftReefLookup() {
        poseLookupList = new ArrayList<Pose2d>();
        // blue left reefs
        poseLookupList.add(new Pose2d(1.635, 3.392, new Rotation2d(Units.degreesToRadians(60)))); 
        poseLookupList.add(new Pose2d(5.155, 2.883, new Rotation2d(Units.degreesToRadians(120)))); 
        poseLookupList.add(new Pose2d(5.803, 4.060, new Rotation2d(Units.degreesToRadians(180)))); 
        poseLookupList.add(new Pose2d(5.056, 5.154, new Rotation2d(Units.degreesToRadians(240)))); 
        poseLookupList.add(new Pose2d(3.869, 5.167, new Rotation2d(Units.degreesToRadians(300)))); 
        poseLookupList.add(new Pose2d(3.202, 4.004, new Rotation2d(Units.degreesToRadians(360)))); 
        // red left reefs
        poseLookupList.add(new Pose2d(12.428, 2.931, new Rotation2d(Units.degreesToRadians(60)))); 
        poseLookupList.add(new Pose2d(13.708, 2.910, new Rotation2d(Units.degreesToRadians(120)))); 
        poseLookupList.add(new Pose2d(14.348, 4.035, new Rotation2d(Units.degreesToRadians(180)))); 
        poseLookupList.add(new Pose2d(13.708, 5.129, new Rotation2d(Units.degreesToRadians(240))));
        poseLookupList.add(new Pose2d(12.449, 5.140, new Rotation2d(Units.degreesToRadians(300))));
        poseLookupList.add(new Pose2d(11.799, 4.025, new Rotation2d(Units.degreesToRadians(0))));
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
