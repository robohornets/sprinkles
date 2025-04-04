package frc.robot.joysticks;

import java.util.Optional;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.helpers.levelmanager.LevelManager;
import frc.robot.helpers.levelmanager.Levels;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.coral.CommandManagers.HandoffManager;
import frc.robot.subsystems.mechanisms.coral.CommandManagers.InOutCommands.CoralInCommand;
import frc.robot.subsystems.mechanisms.coral.CommandManagers.InOutCommands.CoralOutCommand;
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
        // MARK: A - Left Faster
        joystick.leftTrigger()
            .whileTrue(coralSubsystem.funnelUneven(true))
            .onFalse(
                Commands.run(
                    () -> {
                        coralSubsystem.funnelLeft.set(0.0);
                        coralSubsystem.funnelRight.set(0.0);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: B - Right Faster
        joystick.rightTrigger()
            .whileTrue(coralSubsystem.funnelUneven(false))
            .onFalse(
                Commands.run(
                    () -> {
                        coralSubsystem.funnelLeft.set(0.0);
                        coralSubsystem.funnelRight.set(0.0);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: D - Eject
        joystick.a()
            .whileTrue(
                Commands.run(
                    () -> {
                        coralSubsystem.funnelLeft.set(0.25);
                        coralSubsystem.funnelRight.set(-0.25);
                    }
                )
            )
            .onFalse(
                Commands.run(
                    () -> {
                        coralSubsystem.funnelLeft.set(0.0);
                        coralSubsystem.funnelRight.set(0.0);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: E - Handoff
        joystick.b()
            .onTrue(
                new HandoffManager(coralSubsystem, elevatorSubsystem)
            );

        // MARK: G - Recalibrate Coral
        // This code does not exist yet.

        // MARK: H - Algae In
        joystick.y()
            .whileTrue(
                coralSubsystem.flywheelIn()
            )
            .onFalse(
                Commands.run(
                    () -> {
                        coralSubsystem.flywheelMotor.set(0.0);
                    }
                )
            );

        // MARK: J - Algae L3
        joystick.povDown().onTrue(new LevelManager(Levels.ALGAE_3, elevatorSubsystem, coralSubsystem).goToPreset());


        // MARK: K - Algae L2
        joystick.povDown().onTrue(new LevelManager(Levels.ALGAE_2, elevatorSubsystem, coralSubsystem).goToPreset());


        // MARK: DPAD - Elevator
        joystick.povDown().onTrue(new LevelManager(Levels.LEVEL_1, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povLeft().onTrue(new LevelManager(Levels.LEVEL_2, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povRight().onTrue(new LevelManager(Levels.LEVEL_3, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povUp().onTrue(new LevelManager(Levels.LEVEL_4, elevatorSubsystem, coralSubsystem).goToPreset());
    }    
}
