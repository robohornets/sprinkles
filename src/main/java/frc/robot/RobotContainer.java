// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignOnTheFly;
import frc.robot.commands.AlignOnTheFlyClosest;
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
    
    private void configureBindings() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically

            // NOTE:  This is set to work well onscreen not on a real bot
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftX() * MaxSpeed) // Drive forward with negative X (forward)
                    .withVelocityY(joystick.getLeftY() * MaxSpeed) // Drive left with positive Y (right)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().onTrue(Commands.sequence(
            Commands.runOnce(() -> { 
                Robot.myStringLog.append("onTheFlyTest AlignOnTheFlyClosest joystick.a() before pose: " + drivetrain.getState().Pose);
            }, drivetrain),
            new AlignOnTheFlyClosest("collector", drivetrain)
        ));
        
        // collect top algae by passing in Pose2d location
        joystick.a().onTrue(new AlignOnTheFly(new Pose2d(1.05, 6.4, new Rotation2d(216.0)), drivetrain));

        // collect algae from closest station
        joystick.y().onTrue(new AlignOnTheFlyClosest("collector", drivetrain));
        
        // align right side of closest reef
        joystick.b().onTrue(new AlignOnTheFlyClosest("rightReef", drivetrain));
        
        // align with left side of closed reef
        joystick.x().onTrue(new AlignOnTheFlyClosest("leftReef", drivetrain));

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)))));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        Robot.myStringLog.append("onTheFlyTest getAutonomousCommand: " + autoChooser.getSelected().getName());
        return autoChooser.getSelected();
    }
}
