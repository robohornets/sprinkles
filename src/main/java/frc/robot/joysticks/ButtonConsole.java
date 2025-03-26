package frc.robot.joysticks;

import java.util.Optional;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignOnTheFlyByPose;
import frc.robot.helpers.levelmanager.LevelManager;
import frc.robot.helpers.levelmanager.Levels;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;
import frc.robot.subsystems.mechanisms.climber.ClimberVariables;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;

public class ButtonConsole {
    private final RobotContainer robotContainer;
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralSubsystem coralSubsystem;
    private final AlgaeSubsystem algaeSubsytem;
    Optional<Alliance> ally = DriverStation.getAlliance();
    
    public ButtonConsole(RobotContainer robotContainer, CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem, AlgaeSubsystem algaeSubsystem) {

        this.robotContainer = robotContainer;
        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
        this.algaeSubsytem = algaeSubsystem;
    }

    public void configureBindings() {
        // MARK: Climber
        joystick.leftTrigger().whileTrue(
            Commands.run(
                () -> {
                    //ClimberVariables.alexHonnold.setNeutralMode(NeutralModeValue.Brake);
                    ClimberVariables.alexHonnold.set(-1.0);
                }
            )
        ).onFalse(
            Commands.run(
                () -> {
                    ClimberVariables.alexHonnold.setNeutralMode(NeutralModeValue.Brake);
                    ClimberVariables.alexHonnold.set(0.0);
                }
            )
        );

        joystick.rightTrigger()
            .whileTrue(
                Commands.run(
                    () -> {
                        //ClimberVariables.alexHonnold.setNeutralMode(NeutralModeValue.Brake);
                        ClimberVariables.alexHonnold.set(1.0);
                    }
                )
            ).onFalse(
                Commands.run(
                    () -> {
                        ClimberVariables.alexHonnold.setNeutralMode(NeutralModeValue.Brake);
                        ClimberVariables.alexHonnold.set(0.0);
                    }
                )
            );

        
        joystick.leftBumper()
            .whileTrue(elevatorSubsystem.elevatorUpManual())
            .onFalse(
                Commands.run(
                    () -> {
                        elevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                        elevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                        elevatorSubsystem.elevatorLeft.set(0.0);
                        elevatorSubsystem.elevatorRight.set(0.0);
                    }
                )
            );
    
        joystick.rightBumper()
            .whileTrue(elevatorSubsystem.elevatorDownManual())
            .onFalse(
                Commands.run(
                    () -> {
                        elevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                        elevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                        elevatorSubsystem.elevatorLeft.set(0.0);
                        elevatorSubsystem.elevatorRight.set(0.0);
                    }
                )
            );
        

        joystick.a()
            .whileTrue(algaeSubsytem.flywheelAlgaeIn())
            .onFalse(
                Commands.run(
                    () -> {
                        algaeSubsytem.angleAlgaeMotor.set(0.0);
                        algaeSubsytem.angleAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );
        

        joystick.b()
            .whileTrue(coralSubsystem.flywheelIn())
            .onFalse(
                Commands.run(
                    () -> {
                        // CoralSubsystem.angleMotor.set(-0.015);
                        CoralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        joystick.y().onTrue(new LevelManager(Levels.CORAL_STATION, elevatorSubsystem, coralSubsystem).goToPreset());

        // MARK: DPAD Bindings
        joystick.povDown().onTrue(new LevelManager(Levels.LEVEL_1, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povLeft().onTrue(new LevelManager(Levels.LEVEL_2, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povRight().onTrue(new LevelManager(Levels.LEVEL_3, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povUp().onTrue(new LevelManager(Levels.LEVEL_4, elevatorSubsystem, coralSubsystem).goToPreset());
    }    
}
