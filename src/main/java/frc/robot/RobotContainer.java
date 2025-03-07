// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignOnTheFly;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Build auto chooser. This will find all .auto files in deploy/pathplanner/autos
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    /*
    collector AlignOnTheFly currentPose: Pose2d(Translation2d(X: 1.06, Y: 6.39), Rotation2d(Rads: 2.18, Deg: 124.92))

    top left coral AlignOnTheFly currentPose: Pose2d(Translation2d(X: 3.45, Y: 5.10), Rotation2d(Rads: -1.05, Deg: -60.15))
    top right coral AlignOnTheFly currentPose: Pose2d(Translation2d(X: 5.85, Y: 3.80), Rotation2d(Rads: -3.12, Deg: -178.51))
    right middle coral AlignOnTheFly currentPose: Pose2d(Translation2d(X: 2.67, Y: 3.73), Rotation2d(Rads: -0.01, Deg: -0.29))
    bottom right middle coral AlignOnTheFly currentPose: Pose2d(Translation2d(X: 1.06, Y: 6.39), Rotation2d(Rads: 2.18, Deg: 124.84))
    bottom left middle coral AlignOnTheFly currentPose: Pose2d(Translation2d(X: 3.45, Y: 5.10), Rotation2d(Rads: -1.05, Deg: -60.28))
    left middle coral AlignOnTheFly currentPose: Pose2d(Translation2d(X: 5.14, Y: 5.10), Rotation2d(Rads: -2.11, Deg: -120.65))
    */
    
    private void configureBindings() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftX() * MaxSpeed) // Drive forward with negative X (forward)
                    .withVelocityY(joystick.getLeftY() * MaxSpeed) // Drive left with positive Y (right)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().onTrue(Commands.runOnce(() -> { 
                System.out.println("y getPose: " + drivetrain.getState().Pose);
        }, drivetrain));
                
        // collect algae from closest station
        joystick.y().onTrue(new AlignOnTheFly("collector", drivetrain));

        // align right side of closest reef
        joystick.b().onTrue(new AlignOnTheFly("rightReef", drivetrain));
        
        // align with left side of closed reef
        joystick.x().onTrue(new AlignOnTheFly("leftReef", drivetrain));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
