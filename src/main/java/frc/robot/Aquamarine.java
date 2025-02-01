package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

//import com.pathplanner.lib.PathPlanner;
//import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.commands.FollowPathWithEvents;

//import frc.robot.commands.DriveCommands;

public class Aquamarine {
    
    public static Command driveByTime(CommandSwerveDrivetrain drivetrain, com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric drive){
        return Commands.sequence(
            Commands.print("Starting wait command"),
            drivetrain.applyRequest(
                () -> {
                    return (SwerveRequest) drive.withVelocityX(0.0).withVelocityY(0.0);
                    
                }
            ),
            Commands.waitSeconds(5)
        );
    }

    
}
