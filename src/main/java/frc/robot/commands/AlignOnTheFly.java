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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignOnTheFly extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private PathPlannerPath m_path;
    private PathConstraints constraints;

    public AlignOnTheFly(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        createCoralLookup();
        addRequirements(drivetrain);
    }

    private ArrayList<Pose2d> coralLookup;
    private void createCoralLookup() {
        coralLookup = new ArrayList<Pose2d>();
        coralLookup.add(new Pose2d(3.45, 5.10, new Rotation2d(Units.degreesToRadians(-60.15))));
        coralLookup.add(new Pose2d(5.85, 3.80, new Rotation2d(Units.degreesToRadians(-178.51))));
        coralLookup.add(new Pose2d(2.67, 3.73, new Rotation2d(Units.degreesToRadians(-0.29))));
        coralLookup.add(new Pose2d( 1.06, 6.39, new Rotation2d(Units.degreesToRadians(124.84))));
        coralLookup.add(new Pose2d(3.45, 5.10, new Rotation2d(Units.degreesToRadians(-60.28))));
        coralLookup.add(new Pose2d(5.14, 5.10, new Rotation2d(Units.degreesToRadians(-120.65))));
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_drivetrain.getState().Pose;
        ArrayList<Double> distance = new ArrayList<>();
        for (int i=0; i < coralLookup.size(); i++) {
                Pose2d coralSide = coralLookup.get(i);
                System.out.println(coralSide);
                double newDistance =
                        Math.sqrt(Math.pow(coralSide.getX()-currentPose.getX(),2) 
                        + Math.pow(coralSide.getY()-currentPose.getY(),2));
                distance.add(newDistance);
                System.out.println("newDistance: " + newDistance);        
        }
        Optional<Double> minDistance = distance.stream().min(Comparator.naturalOrder());
        int indexOfMin = -1;
        if (minDistance.isPresent()) {
                indexOfMin = distance.indexOf(minDistance.get());
                System.out.println("indexOfMin: " + indexOfMin);
        }
        System.out.println("AlignOnTheFly currentPose: " + currentPose);
        Pose2d m_destinationPose = coralLookup.get(indexOfMin);
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                currentPose, m_destinationPose);

        constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for
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
                                                                       // using a differential drivetrain, the rotation
                                                                       // will have no
                                                                       // effect.
        );
        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        
        m_path = path;
        // Commands.sequence(
        // Commands.runOnce(()->{
        // System.out.println("y AlignOnTheFly PathPoses:");
        // m_path.getPathPoses().forEach(System.out::println);
        // },m_drivetrain),
        // AutoBuilder.followPath(m_path)
        // );
    }

    @Override
    public void execute() {
        if (m_path != null) {
            Commands.sequence(
                    //Commands.runOnce(() -> {
                        //System.out.println("AlignOnTheFly PathPoses:");
                        //m_path.getPathPoses().forEach(System.out::println);
                    //}, m_drivetrain),
                    AutoBuilder.followPath(m_path),
                    //AutoBuilder.pathfindToPose(m_destinationPose, constraints),
                    Commands.runOnce(() -> {
                        try {
                        //System.out.println("AlignOnTheFly AutoBuilder.getCurrentPose: " + AutoBuilder.getCurrentPose());
                    } catch (Exception ex) {
                        //System.out.println("AlignOnTheFly AutoBuilder.getCurrentPose exception: " + ex.getMessage());
                    }
                    //m_path.getPathPoses().forEach(System.out::println);
                    }, m_drivetrain)
            ).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
