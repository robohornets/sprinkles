package frc.robot.namedcommands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.helpers.levelmanager.LevelManager;
import frc.robot.helpers.levelmanager.Levels;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;

public class RegisterCommands {
    ElevatorSubsystem elevatorSubsystem;
    CoralSubsystem coralSubsystem;

    public RegisterCommands(ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
    }
    
    public void registerCommands() {
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
        );
    }
}
