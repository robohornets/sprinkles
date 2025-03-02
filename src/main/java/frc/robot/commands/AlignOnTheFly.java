package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignOnTheFly extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Pose2d m_destinationPose;
    private PathPlannerPath m_path;
    private PathConstraints constraints;

    public AlignOnTheFly(Pose2d destinationPose, CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_destinationPose = destinationPose;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_drivetrain.getState().Pose;
        System.out.println("AlignOnTheFly currentPose: " + currentPose);
        
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
                    Commands.runOnce(() -> {
                        System.out.println("AlignOnTheFly PathPoses:");
                        m_path.getPathPoses().forEach(System.out::println);
                    }, m_drivetrain),
                    AutoBuilder.followPath(m_path),
                    //AutoBuilder.pathfindToPose(m_destinationPose, constraints),
                    Commands.runOnce(() -> {
                        try {
                        System.out.println("AlignOnTheFly AutoBuilder.getCurrentPose: " + AutoBuilder.getCurrentPose());
                    } catch (Exception ex) {
                        System.out.println("AlignOnTheFly AutoBuilder.getCurrentPose exception: " + ex.getMessage());
                    }
                    m_path.getPathPoses().forEach(System.out::println);
                    }, m_drivetrain)
            ).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
