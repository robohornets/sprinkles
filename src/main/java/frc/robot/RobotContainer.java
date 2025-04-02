// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.namedcommands.RegisterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.helpers.ShuffleboardUtil;
import frc.robot.joysticks.ButtonConsole;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.joysticks.MechBackup;
import frc.robot.joysticks.Debug.DebugButtonConsole;
import frc.robot.joysticks.Debug.DebugJoystick;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;

public class RobotContainer {
    // MARK: Constants
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second

    // MARK: Drive System
    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public static final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public boolean slowRobotSpeed = false;

    public double robotSpeedLimiter = 0.80;

    // MARK: Mechanisms
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final CoralSubsystem coralSubsystem = new CoralSubsystem();
    public final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
    public final RegisterCommands registerCommands = new RegisterCommands(elevatorSubsystem, coralSubsystem);

    // MARK: Inputs
    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController mechanismsJoystick = new CommandXboxController(1);
    private final CommandXboxController buttonConsole = new CommandXboxController(2);
    private final CommandXboxController debugJoystick = new CommandXboxController(3);
    private final CommandXboxController debugButtonConsole = new CommandXboxController(4);

    private final DriverJoystick driverJoystickController = new DriverJoystick(driverJoystick, drivetrain, elevatorSubsystem, coralSubsystem, algaeSubsystem);
    private final MechBackup mechanismsJoystickController = new MechBackup(mechanismsJoystick, drivetrain, elevatorSubsystem, coralSubsystem, algaeSubsystem);
    private final DebugJoystick debugJoystickController = new DebugJoystick(debugJoystick, drivetrain, elevatorSubsystem, coralSubsystem, algaeSubsystem);
    private final ButtonConsole buttonConsoleController = new ButtonConsole(this, buttonConsole, drivetrain, elevatorSubsystem, coralSubsystem, algaeSubsystem);
    private final DebugButtonConsole debugButtonConsoleController = new DebugButtonConsole(this, debugButtonConsole, drivetrain, elevatorSubsystem, coralSubsystem, algaeSubsystem);

    // MARK: Shuffleboard
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

            
    public RobotContainer() {
        // Gets rid of annoying print statements in the console
        DriverStation.silenceJoystickConnectionWarning(true);
        
        // MARK: Named Commands
        // This configures the named commands to access during autonomous mode
        /*NamedCommands.registerCommand("autoL1",
                new LevelManager(Levels.LEVEL_1, elevatorSubsystem, coralSubsystem).goToPreset()
        );
        NamedCommands.registerCommand("autoL2",
                new LevelManager(Levels.LEVEL_2, elevatorSubsystem, coralSubsystem).goToPreset()
        );
        NamedCommands.registerCommand("autoL3",
                new LevelManager(Levels.LEVEL_3, elevatorSubsystem, coralSubsystem).goToPreset()
        );
        NamedCommands.registerCommand("autoL4",
                new LevelManager(Levels.LEVEL_4, elevatorSubsystem, coralSubsystem).goToPreset()
        );
        NamedCommands.registerCommand("coralLevel",
                new LevelManager(Levels.CORAL_STATION, elevatorSubsystem, coralSubsystem).goToPreset()
        );
        NamedCommands.registerCommand("defaultPosition",
                new LevelManager(Levels.DEFAULT_POSITION, elevatorSubsystem, coralSubsystem).goToPreset()
        );
        NamedCommands.registerCommand("eatCoral",
            Commands.sequence(
                coralSubsystem.flywheelIn().withTimeout(0.8),
                
                Commands.runOnce(() -> coralSubsystem.flywheelMotor.set(0.0))
            )
        );
        NamedCommands.registerCommand("stopCoral",
            Commands.run(
                () -> {
                    coralSubsystem.flywheelMotor.set(0.0);
                }
            ) 
        );
        NamedCommands.registerCommand("spitCoral",
            Commands.sequence(
                coralSubsystem.flywheelOut().withTimeout(0.3),
                
                Commands.runOnce(() -> coralSubsystem.flywheelMotor.set(0.0))
            )
        );*/

        registerCommands.registerCommands();

        // MARK: Build Autos
        // Build auto chooser. This will find all .auto files in deploy/pathplanner/autos and add them to Shuffleboard
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        drivetrain.registerTelemetry(logger::telemeterize);

        // MARK: Configure Bindings
        driverJoystickController.configureBindings();
        mechanismsJoystickController.configureBindings();
        buttonConsoleController.configureBindings();
        debugJoystickController.configureBindings();
        debugButtonConsoleController.configureBindings();
        configureBindings();
        configureDefaults();
    }

    private void configureDefaults() {
        coralSubsystem
            .setDefaultCommand( 
                Commands.run(
                    () -> {
                        coralSubsystem.angleMotor.set(coralSubsystem.angleHoldSpeed);
                        if (coralSubsystem.funnelSensor.getDistance(true).getValueAsDouble() < 0.1){
                            coralSubsystem.funnelLeft.set(0);
                            coralSubsystem.funnelRight.set(0);
                        } else {
                            coralSubsystem.funnelLeft.set(-coralSubsystem.funnelSpeed);
                            coralSubsystem.funnelRight.set(coralSubsystem.funnelSpeed);
                        }
                    },
                    coralSubsystem
                )
            );

        elevatorSubsystem
            .setDefaultCommand(
                Commands.run(
                    () -> {
                        elevatorSubsystem.elevatorLeft.set(-0.015);
                        elevatorSubsystem.elevatorRight.set(0.015);
                    },
                    elevatorSubsystem
                )
            );
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> RobotContainer.drive.withVelocityX(-driverJoystick.getLeftY() * RobotContainer.MaxSpeed * (slowRobotSpeed ? 0.7: robotSpeedLimiter))
                .withVelocityY(-driverJoystick.getLeftX() * RobotContainer.MaxSpeed * (slowRobotSpeed ? 0.7: robotSpeedLimiter))
                .withRotationalRate(-driverJoystick.getRightX() * RobotContainer.MaxAngularRate)
            )
        );
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}