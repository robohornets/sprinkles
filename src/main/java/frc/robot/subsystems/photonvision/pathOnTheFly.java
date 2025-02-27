package frc.robot.subsystems.photonvision;

import java.util.List;
import frc.robot.subsystems.photonvision.VisionSubsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrain.OdometryThread;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class pathOnTheFly extends Command{
        private final CommandSwerveDrivetrain m_drivetrain;
        private final Pose2d m_destinationPose;
        private PathPlannerPath m_path; 
        

        public pathOnTheFly(Pose2d destinationPose, CommandSwerveDrivetrain drivetrain){
                m_drivetrain = drivetrain;
                m_destinationPose = destinationPose;
                addRequirements(drivetrain);


        }


    // Create a list of waypoints from poses. Each pose represents one waypoint.
// The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.

@Override
public void initialize(){

//VisionSubsystem visionSubsystem1 = new VisionSubsystem();
//Pose2d pose = visionSubsystem1.getLatestEstimatedPose();
Pose2d pose = m_drivetrain.getPose2d();
// WORK HERE
//Pose2d currentPose = m_drivetrain.Pose2d;

// System.out.println(pose);

List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        pose,
        //new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
);

PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
// PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

// Create the path using the waypoints created above
PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
);

path.preventFlipping = true;
m_path = path;

//AutoBuilder.followPath(path);

}

@Override
public void execute() {
        if(m_path != null){
                Commands.sequence(
                        Commands.runOnce(() ->{
                System.out.println("y AlignOnTheFly PathPoses: ");
                m_path.getPathPoses().forEach(System.out::println);}, m_drivetrain),
                AutoBuilder.followPath(m_path)
        ).schedule();
        }
}

// Prevent the path from being flipped if the coordinates are already correct
//path.preventFlipping = true;


} 
