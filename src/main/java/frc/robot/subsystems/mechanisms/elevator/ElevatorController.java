package frc.robot.subsystems.mechanisms.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorController {

    // public ElevatorController() {
    //     ElevatorSubsystem.disableDownTrigger.whileTrue(Commands.runOnce(() -> ElevatorSubsystem.elevatorDownDisabled = true));
    //     ElevatorSubsystem.disableDownTrigger.whileFalse(Commands.runOnce(() -> ElevatorSubsystem.elevatorDownDisabled = false));
    //     ElevatorSubsystem.disableUpTrigger.whileTrue(Commands.runOnce(() -> ElevatorSubsystem.elevatorUpDisabled = true));
    //     ElevatorSubsystem.disableUpTrigger.whileFalse(Commands.runOnce(() -> ElevatorSubsystem.elevatorUpDisabled = false));
    // }

    public Command elevatorUp() {
        return Commands.run(
            () -> {
                if (Math.abs(ElevatorSubsystem.getElevatorHeight()) <= 65.0) {
                    ElevatorSubsystem.elevatorLeft.set(-ElevatorSubsystem.elevatorUpDownSpeed);
                    ElevatorSubsystem.elevatorRight.set(ElevatorSubsystem.elevatorUpDownSpeed);
                } else {
                    ElevatorSubsystem.elevatorLeft.set(0.0);
                    ElevatorSubsystem.elevatorRight.set(0.0);
                    ElevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                    ElevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                }
            }
        );
    }

    public Command elevatorDown() {
        return Commands.run(
            () -> {
                if (Math.abs(ElevatorSubsystem.elevatorLeft.getPosition().getValueAsDouble()) >= 5.0) {
                    ElevatorSubsystem.elevatorLeft.set(ElevatorSubsystem.elevatorUpDownSpeed);
                    ElevatorSubsystem.elevatorRight.set(-ElevatorSubsystem.elevatorUpDownSpeed);
                } else {
                    ElevatorSubsystem.elevatorLeft.set(0.0);
                    ElevatorSubsystem.elevatorRight.set(0.0);
                    ElevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                    ElevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                }
            }
        );
    }

    public Command elevatorUpSlow() {
        return Commands.run(
            () -> {
                if (Math.abs(ElevatorSubsystem.getElevatorHeight()) <= 65.0) {
                    ElevatorSubsystem.elevatorLeft.set(-ElevatorSubsystem.elevatorUpDownSpeedSlow);
                    ElevatorSubsystem.elevatorRight.set(ElevatorSubsystem.elevatorUpDownSpeedSlow);
                } else {
                    ElevatorSubsystem.elevatorLeft.set(0.0);
                    ElevatorSubsystem.elevatorRight.set(0.0);
                    ElevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                    ElevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                }
            }
        );
    }

    public Command elevatorDownSlow() {
        return Commands.run(
            () -> {
                if (Math.abs(ElevatorSubsystem.getElevatorHeight()) >= 5.0) {
                    ElevatorSubsystem.elevatorLeft.set(ElevatorSubsystem.elevatorUpDownSpeedSlow);
                    ElevatorSubsystem.elevatorRight.set(-ElevatorSubsystem.elevatorUpDownSpeedSlow);
                } else {
                    ElevatorSubsystem.elevatorLeft.set(0.0);
                    ElevatorSubsystem.elevatorRight.set(0.0);
                    ElevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                    ElevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                }
            }
        );
    }

    public Command stopElevator() {
        return Commands.run(
            () -> {
                ElevatorSubsystem.elevatorLeft.set(0.0);
                ElevatorSubsystem.elevatorRight.set(0.0);
            }
        );
    }

    public Command elevatorTestDown() {
        return Commands.run(
            () -> {
                if (Math.abs(ElevatorSubsystem.getElevatorHeight()) >= 5.0) {
                    CoralSubsystem.flywheelMotor.set(0.2);
                } else {
                    CoralSubsystem.flywheelMotor.set(0.0);
                }
            }
        );
    }

    public Command elevatorTestUp() {
        return Commands.run(
            () -> {
                if (Math.abs(ElevatorSubsystem.getElevatorHeight()) <= 55) {
                    CoralSubsystem.flywheelMotor.set(-0.2);
                } else {
                    CoralSubsystem.flywheelMotor.set(0.0);
                }
            }
        );
    }


}
