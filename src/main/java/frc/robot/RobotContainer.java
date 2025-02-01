// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final TalonFX thing1 = new TalonFX(9);
    public final TalonFX thing2 = new TalonFX(10);

    public CANrange canRangeSensor = new CANrange(34);

    Trigger canRangeTrigger = new Trigger(() -> canRangeSensor.getDistance(true).refresh().getValueAsDouble() < 0.2);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public static Boolean disableControllerIn = false;

    public Aquamarine aquamarine;

    public RobotContainer() {

        aquamarine = new Aquamarine();
        
        NamedCommands.registerCommand("driveByTime", Aquamarine.driveByTime(drivetrain, drive));

        // canRangeTrigger.whileTrue(new RunCommand(() -> {
        //     thing1.set(0.1);
        //     thing2.set(-0.1);
        // }));

        canRangeTrigger.onTrue(new RunCommand(() -> {
            // When the sensor detects something, disable the controller
            disableControllerIn = true;
            thing1.set(0.1); // Motor action when sensor is triggered
            thing2.set(-0.1);
        }));
        
        canRangeTrigger.onFalse(new RunCommand(() -> {
            // Wait for 2 seconds and then stop the motors and re-enable the controller
            Commands.sequence(
                new WaitCommand(2.0), // Wait for 2 seconds
                new RunCommand(() -> {
                    thing1.set(0.0);  // Stop motor 1
                    thing2.set(0.0);  // Stop motor 2
                    disableControllerIn = false;  // Re-enable the controller inputs
                    CommandScheduler.getInstance().cancelAll();
                })
            ).schedule();
        }));
        
        
        


        // canRangeTrigger.onFalse(new SequentialCommandGroup(() -> {

        //     thing1.set(0.1);
        //     thing2.set(-0.1);
        // }));

        

        // Build auto chooser. This will find all .auto files in deploy/pathplanner/autos
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        configureBindings();
    }
    


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        // Left Bumper
joystick.leftBumper()
.onTrue(
    Commands.run(
        () -> {
            if (!disableControllerIn) { // Only execute if controller is enabled
                thing1.set(0.0);
                thing2.set(0.0);
            }
        }))
        // .onFalse(
        //     Commands.run(
        //         () -> {
        //             if (!disableControllerIn) { // Only execute if controller is enabled
        //                 thing1.set(0.0);
        //                 thing2.set(0.0);
        //             }
        //         }));
.whileTrue(
    Commands.run(
        () -> {
            if (!disableControllerIn) { // Only execute if controller is enabled
                thing1.set(0.1);
                thing2.set(-0.1);
            }
        }));

// Right Bumper
joystick.rightBumper()
.onTrue(
    Commands.run(
        () -> {
            if (!disableControllerIn) { // Only execute if controller is enabled
                thing1.set(0.0);
                thing2.set(0.0);
            }
        }))
.whileTrue(
    Commands.run(
        () -> {
            if (!disableControllerIn) { // Only execute if controller is enabled
                thing1.set(-0.1);
                thing2.set(0.1);
            }
        }));

        joystick.a()
        .onTrue(
            Commands.run(
        () -> {
                thing1.set(0.0);
                thing2.set(0.0);
                disableControllerIn = false;
        })
        );



        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
