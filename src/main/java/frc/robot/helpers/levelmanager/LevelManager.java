package frc.robot.helpers.levelmanager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.coral.CoralController;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorController;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorVariables;
import frc.robot.subsystems.mechanisms.coral.CoralVariables;

public class LevelManager {
    private final Levels selectedLevel;
    private final ElevatorVariables elevatorSubsystem;
    private final CoralVariables coralSubsystem;

    public LevelManager(Levels selectedLevel, ElevatorVariables elevatorSubsystem, CoralVariables coralSubsystem) {
        this.selectedLevel = selectedLevel;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
    }

    /**
     * Executes the correct preset height and angle based on the selected level.
     */
    public Command goToPreset() {
        double targetHeight = 0.0;
        double targetAngle = 0.0;

        // Assign values based on the selected level
        switch (selectedLevel) {
            case LEVEL_1:
                targetHeight = 5.0;
                targetAngle = 0.56;
                break;

            case LEVEL_2:
                targetHeight = 15.0;
                targetAngle = 0.56;
                break;

            case LEVEL_3:
                targetHeight = 33.0;
                targetAngle = 0.56;
                break;

            case LEVEL_4:
                targetHeight = 65.0;
                targetAngle = 0.52;
                break;

            case DEFAULT_POSITION:
                targetHeight = 0.0;
                targetAngle = 0.82;
                break;

            case CORAL_STATION:
                targetHeight = 5.0;
                targetAngle = 0.56;
                break;

            default:
                break;
        }

        // Run both the coral angle and elevator height commands
        return Commands.parallel(
            new CoralSubsystem(targetAngle, coralSubsystem),
            new ElevatorSubsystem(targetHeight, elevatorSubsystem)
        );
    }
}
