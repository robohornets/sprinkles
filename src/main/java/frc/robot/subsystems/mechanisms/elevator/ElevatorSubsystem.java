package frc.robot.subsystems.mechanisms.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    public final double elevatorUpDownSpeed = 0.4;//0.65;
    public final double elevatorUpDownSpeedSlow = 0.1;

    public static final double minHeight = 0.0; 
    public static final double maxHeight = 58.0;

    public final TalonFX elevatorLeft = new TalonFX(10);
    public final TalonFX elevatorRight = new TalonFX(9);

    public double elevatorEncoderOffset = 0.0;

    public void resetElevatorEncoder() {
        // elevatorLeft.setPosition(0.0);
        elevatorEncoderOffset = elevatorLeft.getPosition().getValueAsDouble();
    }

    public double getElevatorHeight() {
        return elevatorLeft.getPosition().getValueAsDouble();
    }

    public Command elevatorUp() {
        return Commands.run(
            () -> {
                if (Math.abs(elevatorLeft.getPosition().getValueAsDouble()) <= 50.0) {
                    elevatorLeft.set(-elevatorUpDownSpeed);
                    elevatorRight.set(elevatorUpDownSpeed);
                    CommandScheduler.getInstance().cancelAll();
                } else {
                    elevatorLeft.set(-0.015);
                    elevatorRight.set(0.015);
                }
            }
        );
    }

    public Command elevatorDown() {
        return Commands.run(
            () -> {
                if (Math.abs(elevatorLeft.getPosition().getValueAsDouble()) >= 5.0) {
                    elevatorLeft.set(elevatorUpDownSpeed);
                    elevatorRight.set(-elevatorUpDownSpeed);
                } else {
                    elevatorLeft.set(-0.015);
                    elevatorRight.set(0.015);
                }
            }
        );
    }

    public Command elevatorUpManual() {
        return Commands.run(
            () -> {
                elevatorLeft.set(-elevatorUpDownSpeed);
                elevatorRight.set(elevatorUpDownSpeed);
            }
        );
    }

    public Command elevatorDownManual() {
        return Commands.run(
            () -> {
                elevatorLeft.set(elevatorUpDownSpeed);
                elevatorRight.set(-elevatorUpDownSpeed);
            }
        );
    }

    public Command elevatorUpSlow() {
        return Commands.run(
            () -> {
                if (Math.abs(elevatorLeft.getPosition().getValueAsDouble()) <= 65.0) {
                    elevatorLeft.set(-elevatorUpDownSpeedSlow);
                    elevatorRight.set(elevatorUpDownSpeedSlow);
                } else {
                    elevatorLeft.set(-0.015);
                    elevatorRight.set(0.015);
                }
            }
        );
    }

    public Command elevatorDownSlow() {
        return Commands.run(
            () -> {
                if (Math.abs(elevatorLeft.getPosition().getValueAsDouble()) >= 5.0) {
                    elevatorLeft.set(elevatorUpDownSpeedSlow);
                    elevatorRight.set(-elevatorUpDownSpeedSlow);
                } else {
                    elevatorLeft.set(-0.015);
                    elevatorRight.set(0.015);
                }
            }
        );
    }

    public Command stopElevator() {
        return Commands.run(
            () -> {
                elevatorLeft.set(-0.015);
                elevatorRight.set(0.015);
            }
        );
    }
}
