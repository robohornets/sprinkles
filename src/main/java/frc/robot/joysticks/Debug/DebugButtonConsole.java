package frc.robot.joysticks.Debug;

import java.util.Optional;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.helpers.levelmanager.LevelManager;
import frc.robot.helpers.levelmanager.Levels;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.coral.CommandManagers.CoralAngleManager;
import frc.robot.subsystems.mechanisms.coral.CommandManagers.HandoffManager;
import frc.robot.subsystems.mechanisms.coral.CommandManagers.InOutCommands.CoralInCommand;
import frc.robot.subsystems.mechanisms.coral.CommandManagers.InOutCommands.CoralOutCommand;
import frc.robot.subsystems.mechanisms.coral.CommandManagers.InOutCommands.FunnelInCommand;
import frc.robot.subsystems.mechanisms.coral.CommandManagers.InOutCommands.FunnelOutCommand;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;

public class DebugButtonConsole {
    private final RobotContainer robotContainer;
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralSubsystem coralSubsystem;
    private final AlgaeSubsystem algaeSubsytem;
    Optional<Alliance> ally = DriverStation.getAlliance();
    
    public DebugButtonConsole(RobotContainer robotContainer, CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem, AlgaeSubsystem algaeSubsystem) {

        this.robotContainer = robotContainer;
        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
        this.algaeSubsytem = algaeSubsystem;
    }

    public void configureBindings() {
        // MARK: A Button
        joystick.leftTrigger()
            .onTrue(
                new CoralOutCommand(coralSubsystem)
            );

        // MARK: B Button
        joystick.rightTrigger()
            .onTrue(
                new CoralInCommand(coralSubsystem)
            );

        
        joystick.leftBumper()
            .whileTrue(elevatorSubsystem.elevatorUpManual())
            .onFalse(
                Commands.run(
                    () -> {
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
                        elevatorSubsystem.elevatorLeft.set(0.0);
                        elevatorSubsystem.elevatorRight.set(0.0);
                    }
                )
            );
        
        
        // E Button
        joystick.b()
            .onTrue(
                new FunnelInCommand(coralSubsystem)
            );
        
        // D Button
        joystick.a()
            .onTrue(
                new InstantCommand(() -> {
                    new FunnelOutCommand(coralSubsystem).schedule();
                    new CoralInCommand(coralSubsystem).schedule();
                })
            );

        joystick.x()
            .onTrue(
                new CoralInCommand(coralSubsystem)
            );

        // F Button
        joystick.y().onTrue(
            Commands.run(
                () -> {
                    coralSubsystem.funnelLeft.set(0.25);
                    coralSubsystem.funnelRight.set(-0.25);
                }
            ).withTimeout(1.5)
        );

        // MARK: DPAD Bindings
        joystick.povDown().onTrue(
            new HandoffManager(coralSubsystem, elevatorSubsystem)
        );
        joystick.povLeft().onTrue(new LevelManager(Levels.LEVEL_2, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povRight().onTrue(new LevelManager(Levels.LEVEL_3, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povUp().onTrue(new LevelManager(Levels.LEVEL_4, elevatorSubsystem, coralSubsystem).goToPreset());
    }    
}
