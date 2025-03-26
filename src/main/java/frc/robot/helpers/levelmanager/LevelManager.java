package frc.robot.helpers.levelmanager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.coral.CoralAngleManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;

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
                targetHeight = 2.0;
                targetAngle = 0.4;
                krakenTargetAngle = 5.0;
                // kraken angle : 5
                break;

            case LEVEL_2:
                targetHeight = 15.17;
                targetAngle = 0.372;
                krakenTargetAngle = 5.8;
                // kraken angle : 5.8
                break;

            case LEVEL_3:
                targetHeight = 33.690;
                targetAngle = 0.372;
                krakenTargetAngle = 5.7;
                // kraken angle : 5.7
                break;

            case LEVEL_4:
                targetHeight = 65.049;
                targetAngle = 0.385;
                krakenTargetAngle = 5.8;
                // kraken angle : 5.8
                break;

            case DEFAULT_POSITION:
                targetHeight = 0.0;
                targetAngle = 0.82;
                krakenTargetAngle = 0.0;
                // kraken angle : 0.0
                break;

            case CORAL_STATION:
                targetHeight = 8.1;
                targetAngle = 0.55;
                krakenTargetAngle = 2.4;
                // kraken angle : 2.4
                break;

            default:
                break;
        }

        // Run both the coral angle and elevator height commands
        return Commands.parallel(
            new CoralAngleManager(targetAngle, krakenTargetAngle, coralSubsystem),
            new ElevatorHeightManager(targetHeight, elevatorSubsystem)
        );
    }
}
