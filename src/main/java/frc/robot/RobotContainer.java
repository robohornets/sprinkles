// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignOnTheFly;
import frc.robot.commands.Destinations;
import frc.robot.generated.TunerConstants;
import frc.robot.joysticks.DebugJoystick;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.namedcommands.AutoNamedCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.coral.CoralController;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.coral.CoralVariables;
import frc.robot.subsystems.mechanisms.elevator.ElevatorAutoHeight;
import frc.robot.subsystems.mechanisms.elevator.ElevatorController;
import frc.robot.subsystems.mechanisms.elevator.ElevatorVariables;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class RobotContainer {
    // MARK: Constants
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second


    // MARK: Drive System
    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Telemetry logger = new Telemetry(MaxSpeed);


    // MARK: Triggers
    public CANrange canRangeSensor = new CANrange(34);
    Trigger canRangeTrigger = new Trigger(() -> canRangeSensor.getDistance(true).refresh().getValueAsDouble() < 0.2);

    // Triggers for Button Console
    final Trigger positionATrigger = new Trigger(
            () -> Math.round(this.joystick2.getLeftX() * 10) / 10 == 0.1
                    && Math.round(this.joystick2.getLeftY() * 10) / 10 == 0.1);

    // MARK: Mechanisms
    private final ElevatorController elevator = new ElevatorController();
    public static final CoralController coral = new CoralController();

    public final ElevatorVariables elevatorSubsystem = new ElevatorVariables();
    public final CoralVariables coralSubsystem = new CoralVariables();

    // MARK: Inputs
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    private final DriverJoystick driverJoystick = new DriverJoystick(joystick, drivetrain, elevator, elevatorSubsystem, coral, coralSubsystem);
    private final DebugJoystick debugJoystick = new DebugJoystick(joystick, drivetrain, elevator, elevatorSubsystem, coral, coralSubsystem);


    // MARK: Shuffleboard
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private GenericEntry additionalAngleSpeed = Shuffleboard.getTab("Coral")
            .add("Additional Angle Speed", 0.1)
            .withWidget("Number Slider")
            .withProperties(Map.of("min", -1.0, "max", 1.0)) // adjust min and max as needed
            .getEntry();

            
    public RobotContainer() {
        // Gets rid of annoying print statements in the console
        DriverStation.silenceJoystickConnectionWarning(true);
        
        // MARK: Named Commands
        NamedCommands.registerCommand("driveByTime",
                Commands.sequence(
                        Commands.print("Starting wait command"),
                        drivetrain.applyRequest(
                                () -> {
                                    return (SwerveRequest) drive.withVelocityX(0.0)
                                            .withVelocityY(0.0);

                                }).withTimeout(5)));
        NamedCommands.registerCommand("driveByTimeAlt",
                new SequentialCommandGroup(
                        new PrintCommand("Starting drive command"), // This is to make sure we
                                                                    // see this in the log
                        drivetrain.applyRequest(() -> {
                            return drive.withVelocityX(0.0).withVelocityY(0.0);
                        }).withTimeout(5),
                        new PrintCommand("Drive command finished"),
                        // new WaitCommand(5), // Wait for 5 seconds
                        new PrintCommand("Clearing commands")
                // new RunCommand(() -> {
                // CommandScheduler.getInstance().cancelAll();
                // })
                ));

        NamedCommands.registerCommand("driveByTimeAltAlt",
                new SequentialCommandGroup(
                        new PrintCommand("Starting drive command"), // This is to make sure we
                                                                    // see this in the log
                        drivetrain.applyRequest(() -> {
                            return drive.withVelocityX(0.0).withVelocityY(0.0);
                        }).withTimeout(2),
                        new PrintCommand("Drive command finished"),
                        // new WaitCommand(5), // Wait for 5 seconds
                        new PrintCommand("Clearning commands")
                // new RunCommand(() -> {
                // CommandScheduler.getInstance().cancelAll();
                // })
                ));

        // Build auto chooser. This will find all .auto files in deploy/pathplanner/autos
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Trigger configuration for joysticks
        positionATrigger.onTrue(new ElevatorAutoHeight(40.0, elevatorSubsystem));

        drivetrain.registerTelemetry(logger::telemeterize);

        // MARK: Configure Bindings
        driverJoystick.configureBindings();
        // configureBindings();
    }

    private void configureBindings() {
        
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
