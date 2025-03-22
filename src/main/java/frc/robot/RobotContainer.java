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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignOnTheFlyClosest;
import frc.robot.commands.AlignOnTheFlyByPose;
import frc.robot.commands.Destinations;
import frc.robot.generated.TunerConstants;
import frc.robot.helpers.ShuffleboardUtil;
import frc.robot.helpers.levelmanager.LevelManager;
import frc.robot.helpers.levelmanager.Levels;
import frc.robot.joysticks.ButtonConsole;
import frc.robot.joysticks.DebugJoystick;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.joysticks.MechBackup;
import frc.robot.namedcommands.AutoNamedCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.coral.CoralController;
import frc.robot.subsystems.mechanisms.algae.AlgaeController;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;
import frc.robot.subsystems.mechanisms.coral.CoralAngleManager;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorController;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
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


    public static final CANrange elevatorDownSensor = new CANrange(34);

    public static Trigger resetElevatorEncoderTrigger = new Trigger(
        () -> elevatorDownSensor.getDistance(true).refresh().getValueAsDouble() <= 0.2
    );

    
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
    
    
        // MARK: Triggers
        public CANrange canRangeSensor = new CANrange(34);
        Trigger canRangeTrigger = new Trigger(() -> canRangeSensor.getDistance(true).refresh().getValueAsDouble() < 0.2);
    
        public boolean camerasEnabled = true;

        public boolean slowRobotSpeed = false;

        public double robotSpeedLimiter = 0.80;
    
        // MARK: Mechanisms
        private final ElevatorController elevator = new ElevatorController();
        public static final CoralController coral = new CoralController();
        public static final AlgaeController algae = new AlgaeController();
    
        public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        public final CoralSubsystem coralSubsystem = new CoralSubsystem();
        public final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
    
        // MARK: Inputs
        private final CommandXboxController driverJoystick = new CommandXboxController(0);
        private final CommandXboxController mechanismsJoystick = new CommandXboxController(1);
        private final CommandXboxController buttonConsole = new CommandXboxController(2);
        private final CommandXboxController debugJoystick = new CommandXboxController(3);
    
        private final DriverJoystick driverJoystickController = new DriverJoystick(driverJoystick, drivetrain, elevator, elevatorSubsystem, coral, coralSubsystem, algae, algaeSubsystem);
        private final MechBackup mechanismsJoystickController = new MechBackup(mechanismsJoystick, drivetrain, elevator, elevatorSubsystem, coral, coralSubsystem, algae, algaeSubsystem);
        private final DebugJoystick debugJoystickController = new DebugJoystick(debugJoystick, drivetrain, elevator, elevatorSubsystem, coral, coralSubsystem, algae, algaeSubsystem);
        private final ButtonConsole buttonConsoleController = new ButtonConsole(this, buttonConsole, drivetrain, elevator, elevatorSubsystem, coral, coralSubsystem, algae, algaeSubsystem);
    
        // MARK: Shuffleboard
        /* Path follower */
        private final SendableChooser<Command> autoChooser;
    
                
        public RobotContainer() {
            // Gets rid of annoying print statements in the console
            DriverStation.silenceJoystickConnectionWarning(true);
            
            // MARK: Named Commands
            NamedCommands.registerCommand("autoL1",
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
                    coral.flywheelIn().withTimeout(0.8),
                    
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
                    coral.flywheelOut().withTimeout(0.3),
                    
                    Commands.runOnce(() -> coralSubsystem.flywheelMotor.set(0.0))
                )
            );

            resetElevatorEncoderTrigger
            .onTrue(
                Commands.run(
                    () -> {
                        elevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                        elevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                    }
                )
            )
            .whileTrue(
                Commands.run(
                    () -> {
                        ShuffleboardUtil.put("Elevator Encoder", true);
                        elevatorSubsystem.resetElevatorEncoder();
                        //elevatorSubsystem.elevatorLeft.setPosition(0.0);
                    }
                )
            )
            .whileFalse(
                Commands.run(
                    () -> {
                        ShuffleboardUtil.put("Elevator Encoder", false);
                    }
                )
            );
    
    
            // Build auto chooser. This will find all .auto files in deploy/pathplanner/autos
            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Mode", autoChooser);  
            ShuffleboardUtil.put("Auto Selector Backup", autoChooser);
    
            drivetrain.registerTelemetry(logger::telemeterize);
    
            // MARK: Configure Bindings
            driverJoystickController.configureBindings();
            mechanismsJoystickController.configureBindings();
            buttonConsoleController.configureBindings();
            debugJoystickController.configureBindings();
            configureBindings();
            configureDefaults();
            elevatorSubsystem.configureTriggers();
        }
    
        // private void configureBindings() {
        //     drivetrain.setDefaultCommand(
        //         drivetrain.applyRequest(() -> RobotContainer.drive.withVelocityX(-driverJoystick.getLeftY() * RobotContainer.MaxSpeed * 0.5)
        //             .withVelocityY(-driverJoystick.getLeftX() * RobotContainer.MaxSpeed * 0.5)
        //             .withRotationalRate(-driverJoystick.getRightX() * RobotContainer.MaxAngularRate)
        //         )
        //     );
        // }

        private void configureDefaults() {
            coralSubsystem.setDefaultCommand(
                Commands.run(
                    () -> {
                        coralSubsystem.flywheelMotor.set(-0.06);
                        coralSubsystem.angleMotor.set(-CoralSubsystem.angleHoldSpeed);
                    },
                    coralSubsystem
                )
            );

            algaeSubsystem.setDefaultCommand(
                Commands.run(
                    () -> {
                        if (algae.getAlgaeAngle() < -8.0) {
                            algaeSubsystem.flywheelAlgaeMotor.set(-0.2);
                        }
                        else {
                            algaeSubsystem.flywheelAlgaeMotor.set(0.0);
                        }
                        //CommandScheduler.getInstance().cancelAll();
                    },
                    algaeSubsystem
                )
            );
        }
    
        private void configureBindings() {
            //if(useFieldCentric){
            drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> RobotContainer.drive.withVelocityX(-driverJoystick.getLeftY() * RobotContainer.MaxSpeed * (slowRobotSpeed ? 0.7: robotSpeedLimiter))
                    .withVelocityY(-driverJoystick.getLeftX() * RobotContainer.MaxSpeed * (slowRobotSpeed ? 0.7: robotSpeedLimiter))
                    .withRotationalRate(-driverJoystick.getRightX() * RobotContainer.MaxAngularRate)
                )
            );//}
            // else{
            //     drivetrain.setDefaultCommand(
            //         drivetrain.applyRequest(() -> RobotContainer.driveRobotCentric.withVelocityX(-driverJoystick.getLeftY() * RobotContainer.MaxSpeed * 0.5)
            //             .withVelocityY(-driverJoystick.getLeftX() * RobotContainer.MaxSpeed * 0.5)
            //             .withRotationalRate(-driverJoystick.getRightX() * RobotContainer.MaxAngularRate)
            //         )
            //     );
            
            // }
        }
    
        public Command applyHoldCurrent() {
            return Commands.run(
                () -> {
                    coralSubsystem.angleMotor.set(-0.015);
                    elevatorSubsystem.elevatorLeft.set(-0.015);
                    elevatorSubsystem.elevatorRight.set(0.015); 
                }
            );
        }
    
        public Command getAutonomousCommand() {
            /* Run the path selected from the auto chooser */
            return autoChooser.getSelected();
        }
    
    //     public static void setUseFieldCentric(boolean useFieldCentric) {
    //         RobotContainer.useFieldCentric = useFieldCentric;
    // }
}