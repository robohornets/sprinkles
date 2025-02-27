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
import frc.robot.generated.TunerConstants;
import frc.robot.namedcommands.AutoNamedCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.coral.CoralController;
import frc.robot.subsystems.mechanisms.coral.CoralVariables;
import frc.robot.subsystems.mechanisms.elevator.ElevatorController;
import frc.robot.subsystems.mechanisms.elevator.ElevatorVariables;
import frc.robot.subsystems.photonvision.pathOnTheFly;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

//import frc.robot.subsystems.photonvision.pathOnTheFly;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

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
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // private final ElevatorController elevator = new ElevatorController();
    private final CoralController coral = new CoralController();

    // public final TalonFX thing1 = new TalonFX(9);
    // public final TalonFX thing2 = new TalonFX(10);
    public final TalonFX elevatorLeft = new TalonFX(9);
    public final TalonFX elevatorRight = new TalonFX(10);

    public CANrange canRangeSensor = new CANrange(34);

    Trigger canRangeTrigger = new Trigger(() -> canRangeSensor.getDistance(true).refresh().getValueAsDouble() < 0.2);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public static Boolean disableControllerIn = false;

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // NamedCommands.registerCommand("driveByTime",
        // Aquamarine.driveByTime(drivetrain, drive));
        NamedCommands.registerCommand("driveByTime",
                Commands.sequence(
                        Commands.print("Starting wait command"),
                        drivetrain.applyRequest(
                                () -> {
                                    return (SwerveRequest) drive.withVelocityX(0.0).withVelocityY(0.0);

                                }).withTimeout(5)));
        NamedCommands.registerCommand("driveByTimeAlt",
                new SequentialCommandGroup(
                        new PrintCommand("Starting drive command"), // This is to make sure we see this in the log
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
                        new PrintCommand("Starting drive command"), // This is to make sure we see this in the log
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

        // Build auto chooser. This will find all .auto files in
        // deploy/pathplanner/autos
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.5) // Drive
                                                                                                         // forward with
                                                                                                         // negative Y
                                                                                                         // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        joystick.a().onTrue(new pathOnTheFly(new Pose2d(0.0, 2.0, Rotation2d.fromDegrees(0)), drivetrain));

        // reset the field-centric heading on left bumper press
        joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
