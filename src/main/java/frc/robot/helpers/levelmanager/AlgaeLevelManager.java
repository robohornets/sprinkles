package frc.robot.helpers.levelmanager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanisms.algae.AlgaeAngleManager;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;

public class AlgaeLevelManager {
    private final AlgaeLevels selectedAlgaeLevel;
    private final ElevatorSubsystem elevatorSubsystem;
    private final AlgaeSubsystem algaeSubsystem;

    public AlgaeLevelManager(AlgaeLevels selectedAlgaeLevel, ElevatorSubsystem elevatorSubsystem, AlgaeSubsystem algaeSubsystem) {
        this.selectedAlgaeLevel = selectedAlgaeLevel;
        this.elevatorSubsystem = elevatorSubsystem;
        this.algaeSubsystem = algaeSubsystem;
    }
// get coral limits -> get heights
    /**
     * Executes the correct preset height and angle based on the selected level.
     */
    public Command algaeGoToPreset() {
        double targetHeight = 0.0;
        double targetAngle = 0.0;
        // max Coral: 0.693
        // min coral: 0.262

        // Assign values based on the selected level
        switch (selectedAlgaeLevel) {
            case A_LEVEL_1:
                targetHeight = 5.0;
                targetAngle = 0.56;
                break;

            case A_LEVEL_2:
                targetHeight = 15.0;
                targetAngle = 0.56;
                break;

            case A_LEVEL_3:
                targetHeight = 33.0;
                targetAngle = 0.56;
                break;

            case A_LEVEL_4:
                targetHeight = 65.0;
                targetAngle = 0.52;
                break;

            case A_DEFAULT_POSITION:
                targetHeight = 0.0;
                targetAngle = 0.82;
                break;

            case A_CORAL_STATION:
                targetHeight = 4.0;
                targetAngle = 0.75;
                break;

            default:
                break;
        }

        // Run both the coral angle and elevator height commands
        return Commands.parallel(
            new AlgaeAngleManager(targetAngle, algaeSubsystem),
            new ElevatorHeightManager(targetHeight, elevatorSubsystem)
        );
    }
    
}
