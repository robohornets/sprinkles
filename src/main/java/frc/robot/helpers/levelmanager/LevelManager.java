package frc.robot.helpers.levelmanager;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.coral.CommandManagers.CoralAngleManager;

public class LevelManager {
    private final Levels selectedLevel;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralSubsystem coralSubsystem;

    public LevelManager(Levels selectedLevel, ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem) {
        this.selectedLevel = selectedLevel;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
    }
// get coral limits -> get heights
    /**
     * Executes the correct preset height and angle based on the selected level.
     */
    public Command goToPreset() {
        double targetHeight = 0.0;
        double targetAngle = 0.0;
        double krakenTargetAngle = 0.0;
        // max Coral: 0.693
        // min coral: 0.262

        // Assign values based on the selected level
        switch (selectedLevel) {
            case LEVEL_1:
                targetHeight = 0.0;
                krakenTargetAngle = 0.0;
                break;

            case LEVEL_2:
                targetHeight = 15.88;
                krakenTargetAngle = -3.77;
                break;

            case LEVEL_3:
                targetHeight = 32;
                krakenTargetAngle = -3.77;
                break;

            case LEVEL_4:
                targetHeight = 64;
                krakenTargetAngle = -3.86;
                break;

            case DEFAULT_POSITION:
                targetHeight = 0.0;
                krakenTargetAngle = 0.0;
                break;

            case CORAL_STATION:
                targetHeight = 2.1;
                krakenTargetAngle = 2.4;
                break;

            case ZERO:
                targetHeight = 0.0;
                krakenTargetAngle = 2.4;
                break;

            default:
                break;
        }

        // Run both the coral angle and elevator height commands
        return Commands.parallel(
            new CoralAngleManager(krakenTargetAngle, coralSubsystem),
            new ElevatorHeightManager(targetHeight, elevatorSubsystem)
        );
    }
}
