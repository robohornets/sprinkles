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
import frc.robot.generated.TunerConstants;
import frc.robot.namedcommands.AutoNamedCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.coral.CoralController;
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
    private final ElevatorController elevator = new ElevatorController();
    private final CoralController coral = new CoralController();

    public final ElevatorVariables elevatorSubsystem = new ElevatorVariables();

    // public final TalonFX thing1 = new TalonFX(9);
    // public final TalonFX thing2 = new TalonFX(10);
    public final TalonFX elevatorLeft = new TalonFX(9);
    public final TalonFX elevatorRight = new TalonFX(10);

    public CANrange canRangeSensor = new CANrange(34);

    Trigger canRangeTrigger = new Trigger(() -> canRangeSensor.getDistance(true).refresh().getValueAsDouble() < 0.2);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public static Boolean disableControllerIn = false;

    // Triggers for Button Console
    final Trigger positionATrigger = new Trigger(
        () -> Math.round(this.joystick2.getLeftX() * 10)/10 == 0.1 && Math.round(this.joystick2.getLeftY() * 10)/10 == 0.1
    );

    private GenericEntry additionalAngleSpeed = Shuffleboard.getTab("Coral")
            .add("Additional Angle Speed", 0.1)
            .withWidget("Number Slider")
            .withProperties(Map.of("min", -1.0, "max", 1.0)) // adjust min and max as needed
            .getEntry();

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

        positionATrigger.onTrue(getAutonomousCommand());

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

        // reset the field-centric heading on left bumper press
        // joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.leftBumper()
                // .whileTrue(elevator.elevatorDown())
                // .onFalse(elevator.stopElevator());
                .whileTrue(elevator.elevatorDown())
                .onFalse(Commands.run(
                        () -> {
                            ElevatorVariables.elevatorLeft.set(0.0);
                            ElevatorVariables.elevatorRight.set(0.0);
                            ElevatorVariables.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                            ElevatorVariables.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                            CommandScheduler.getInstance().cancelAll();
                        }));

        joystick.rightBumper()
                // .whileTrue(elevator.elevatorUp())
                // .onFalse(elevator.stopElevator());
                .whileTrue(elevator.elevatorUp())
                .onFalse(Commands.run(
                        () -> {
                            ElevatorVariables.elevatorLeft.set(0.0);
                            ElevatorVariables.elevatorRight.set(0.0);
                            ElevatorVariables.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                            ElevatorVariables.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                            CommandScheduler.getInstance().cancelAll();

                        }));

        joystick.a()
                .whileTrue(coral.flywheelOut())
                .onFalse(Commands.run(
                        () -> {
                            CoralVariables.flywheelMotor.set(0.0);
                            CoralVariables.flywheelMotor.setNeutralMode(NeutralModeValue.Coast);
                            CommandScheduler.getInstance().cancelAll();
                        }));

        joystick.b()
                .whileTrue(coral.flywheelIn())
                .onFalse(Commands.run(
                        () -> {
                            CoralVariables.flywheelMotor.set(0.0);
                            CoralVariables.flywheelMotor.setNeutralMode(NeutralModeValue.Coast);
                            CommandScheduler.getInstance().cancelAll();
                        }));

        joystick.leftTrigger()
                .whileTrue(coral.angleDown())
                .onFalse(Commands.run(
                        () -> {
                            CoralVariables.angleMotor.set(0.0);
                            CoralVariables.angleMotor.setNeutralMode(NeutralModeValue.Brake);
                            CommandScheduler.getInstance().cancelAll();
                        }));
        joystick.rightTrigger()
                .whileTrue(coral.angleUp())
                .onFalse(Commands.run(
                        () -> {
                            CoralVariables.angleMotor.set(0.0);
                            CoralVariables.angleMotor.setNeutralMode(NeutralModeValue.Brake);
                            CommandScheduler.getInstance().cancelAll();
                        }));

        joystick2.a()
                .onTrue(Commands.runOnce(() -> {
                    System.out.println("y getPose: " + drivetrain.getState().Pose);
                }, drivetrain));

        // collect algae
        joystick2.y().onTrue(
                new AlignOnTheFly(new Pose2d(1.05, 6.4, new Rotation2d(Units.degreesToRadians(125.0))), drivetrain));

        // align with top left coral
        joystick2.x().onTrue(
                new AlignOnTheFly(new Pose2d(3.46, 5.088, new Rotation2d(Units.degreesToRadians(300.0))), drivetrain));

        // align with top right coral
        joystick2.b().onTrue(
                new AlignOnTheFly(new Pose2d(5.142, 5.088, new Rotation2d(Units.degreesToRadians(240.0))), drivetrain));

        joystick2.povDown().onTrue(new ElevatorAutoHeight(0.0, elevatorSubsystem));
        joystick2.povLeft().onTrue(new ElevatorAutoHeight(20.0, elevatorSubsystem));
        joystick2.povRight().onTrue(new ElevatorAutoHeight(40.0, elevatorSubsystem));
        joystick2.povUp().onTrue(new ElevatorAutoHeight(57.0, elevatorSubsystem));

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
