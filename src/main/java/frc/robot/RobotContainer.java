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


    // MARK: Inputs
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public CANrange canRangeSensor = new CANrange(34);


    // MARK: Triggers
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

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.



        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.5)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.5)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        joystick.leftBumper()
                // .whileTrue(elevator.elevatorDown())
                // .onFalse(elevator.stopElevator());
                .whileTrue(elevator.elevatorDown())
                .onFalse(Commands.run(
                        () -> {
                            ElevatorVariables.elevatorLeft.set(-0.015);
                            ElevatorVariables.elevatorRight.set(0.015);
                            ElevatorVariables.elevatorLeft
                                    .setNeutralMode(NeutralModeValue.Brake);
                            ElevatorVariables.elevatorRight
                                    .setNeutralMode(NeutralModeValue.Brake);
                            CommandScheduler.getInstance().cancelAll();
                        }));

        joystick.rightBumper()
                // .whileTrue(elevator.elevatorUp())
                // .onFalse(elevator.stopElevator());
                .whileTrue(elevator.elevatorUp())
                .onFalse(Commands.run(
                        () -> {
                            ElevatorVariables.elevatorLeft.set(-0.015);
                            ElevatorVariables.elevatorRight.set(0.015);
                            ElevatorVariables.elevatorLeft
                                    .setNeutralMode(NeutralModeValue.Brake);
                            ElevatorVariables.elevatorRight
                                    .setNeutralMode(NeutralModeValue.Brake);
                            CommandScheduler.getInstance().cancelAll();

                        }));

        joystick.a()
                .whileTrue(coral.flywheelOut())
                .onFalse(Commands.run(
                        () -> {
                            CoralVariables.flywheelMotor.set(0.0);
                            CoralVariables.flywheelMotor
                                    .setNeutralMode(NeutralModeValue.Coast);
                            CommandScheduler.getInstance().cancelAll();
                        }));

        joystick.b()
                .whileTrue(coral.flywheelIn())
                .onFalse(Commands.run(
                        () -> {
                            // ElevatorVariables.elevatorLeft.setNeutralMode(NeutralModeValue.Coast);
                            // ElevatorVariables.elevatorRight.setNeutralMode(NeutralModeValue.Coast);
                            CoralVariables.flywheelMotor.set(0.0);
                            CoralVariables.flywheelMotor
                                    .setNeutralMode(NeutralModeValue.Coast);
                            CommandScheduler.getInstance().cancelAll();
                        }));

        joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.x().onTrue(new AlignOnTheFly(Destinations.LEFT_REEF, drivetrain));
        
        
        joystick.leftTrigger()
                .whileTrue(coral.angleDown())
                .onFalse(Commands.run(
                        () -> {
                            CoralVariables.angleMotor.set(-0.015);
                            CoralVariables.angleMotor
                                    .setNeutralMode(NeutralModeValue.Brake);
                            CommandScheduler.getInstance().cancelAll();
                        }));
        joystick.rightTrigger()
                .whileTrue(coral.angleUp())
                .onFalse(Commands.run(
                        () -> {
                            CoralVariables.angleMotor.set(-0.015);
                            CoralVariables.angleMotor
                                    .setNeutralMode(NeutralModeValue.Brake);
                            CommandScheduler.getInstance().cancelAll();
                        }));

        joystick2.a()
                .onTrue(Commands.runOnce(() -> {
                    System.out.println("y getPose: " + drivetrain.getState().Pose);
                }, drivetrain));


        // joystick2.povDown().onTrue(new ElevatorAutoHeight(0.0, elevatorSubsystem));
        joystick2.povLeft().onTrue(new ElevatorAutoHeight(17.0, elevatorSubsystem));
        joystick2.povRight().onTrue(new ElevatorAutoHeight(37.0, elevatorSubsystem));
        joystick2.povUp().onTrue(new ElevatorAutoHeight(57.0, elevatorSubsystem));

        joystick2.povDown().onTrue(new ElevatorAutoHeight(4.0, elevatorSubsystem));
        
        // For coral station - angle: 0.75, elevator: 4.0
        // L1 - angle: , elevator: 
        // L2 - angle: 0.56, elevator: 15
        // L3 - angle: 0.56, elevator: 33
        // L4 - angle: 0.52, elevator: 65

        joystick.povUp().onTrue(new CoralSubsystem(0.82, coralSubsystem));
        joystick.povLeft().onTrue(new CoralSubsystem(0.75, coralSubsystem));
        joystick.povRight().onTrue(new CoralSubsystem(0.6, coralSubsystem));
        joystick.povDown().onTrue(new CoralSubsystem(0.48, coralSubsystem));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
