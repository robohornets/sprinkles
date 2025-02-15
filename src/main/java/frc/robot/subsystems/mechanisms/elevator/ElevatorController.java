package frc.robot.subsystems.mechanisms.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.mechanisms.coral.CoralVariables;

public class ElevatorController {

    public ElevatorController() {
        ElevatorVariables.disableDownTrigger.whileTrue(Commands.runOnce(() -> ElevatorVariables.elevatorDownDisabled = true));
        ElevatorVariables.disableDownTrigger.whileFalse(Commands.runOnce(() -> ElevatorVariables.elevatorDownDisabled = false));
        ElevatorVariables.disableUpTrigger.whileTrue(Commands.runOnce(() -> ElevatorVariables.elevatorUpDisabled = true));
        ElevatorVariables.disableUpTrigger.whileFalse(Commands.runOnce(() -> ElevatorVariables.elevatorUpDisabled = false));
    }

    public Command elevatorUp() {
        return Commands.run(
            () -> {
                if (!ElevatorVariables.elevatorUpDisabled) {
                    ElevatorVariables.elevatorLeft.set(ElevatorVariables.elevatorUpDownSpeed);
                    ElevatorVariables.elevatorRight.set(-ElevatorVariables.elevatorUpDownSpeed);
                }
            }
        );
    }

    public Command elevatorDown() {
        return Commands.run(
            () -> {
                if (!ElevatorVariables.elevatorDownDisabled) {
                    ElevatorVariables.elevatorLeft.set(-ElevatorVariables.elevatorUpDownSpeed);
                    ElevatorVariables.elevatorRight.set(ElevatorVariables.elevatorUpDownSpeed);
                }
            }
        );
    }

    public Command stopElevator() {
        return Commands.run(
            () -> {
                ElevatorVariables.elevatorLeft.set(0.0);
                ElevatorVariables.elevatorRight.set(0.0);
            }
        );
    }

    public Command elevatorTestDown() {
        return Commands.run(
            () -> {
                if (!ElevatorVariables.elevatorDownDisabled) {
                    CoralVariables.flywheelMotor.set(0.2);
                } else {
                    CoralVariables.flywheelMotor.set(0.0);
                }
            }
        );
    }

    public Command elevatorTestUp() {
        return Commands.run(
            () -> {
                if (!ElevatorVariables.elevatorUpDisabled) {
                    CoralVariables.flywheelMotor.set(-0.2);
                } else {
                    CoralVariables.flywheelMotor.set(0.0);
                }
            }
        );
    }
}
