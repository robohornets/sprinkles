package frc.robot.subsystems.mechanisms.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;

public class ElevatorSubsystem extends SubsystemBase {
    public ElevatorSubsystem() {
        disableDownTrigger.whileTrue(Commands.runOnce(() -> elevatorDownDisabled = true));
        disableDownTrigger.whileFalse(Commands.runOnce(() -> elevatorDownDisabled = false));
        disableUpTrigger.whileTrue(Commands.runOnce(() -> elevatorUpDisabled = true));
        disableUpTrigger.whileFalse(Commands.runOnce(() -> elevatorUpDisabled = false));
    }


    public static final double elevatorUpDownSpeed = 0.5;
    public static final double elevatorUpDownSpeedSlow = 0.1;

    public static final TalonFX elevatorLeft = new TalonFX(10);
    public static final TalonFX elevatorRight = new TalonFX(9);

    // Disables all control of the elevator
    public static boolean elevatorDownDisabled = false;
    public static boolean elevatorUpDisabled = false;

    // CANrange sensor and trigger for lower elevator
    //public static final CANrange elevatorDownSensor = new CANrange(34);

    // public static Trigger disableDownTrigger = new Trigger(
    //     () -> Robot.elevatorEncoder.getDistance() >= -1 && Robot.elevatorEncoder.getDistance() <= 0.5
    // );
    
    // public static Trigger disableUpTrigger = new Trigger(
    //     () -> Robot.elevatorEncoder.getDistance() > 5
    // );

    public static Trigger disableDownTrigger = new Trigger(
        () -> elevatorRight.getPosition().getValueAsDouble() <= 0.5
    );
    
    public static Trigger disableUpTrigger = new Trigger(
        () -> elevatorRight.getPosition().getValueAsDouble() >= 5.0
    );

    // **Added Encoder (REV Through Bore in Quadrature Mode)**
    //public static Encoder elevatorEncoder = new Encoder(0, 1);

    public double getElevatorHeight() {
        return elevatorLeft.getPosition().getValueAsDouble();
    }

    public Command elevatorUp() {
        return Commands.run(
            () -> {
                if (Math.abs(elevatorLeft.getPosition().getValueAsDouble()) <= 65.0) {
                    elevatorLeft.set(-elevatorUpDownSpeed);
                    elevatorRight.set(elevatorUpDownSpeed);
                } else {
                    elevatorLeft.set(0.0);
                    elevatorRight.set(0.0);
                    elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                    elevatorRight.setNeutralMode(NeutralModeValue.Brake);
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
                    elevatorLeft.set(0.0);
                    elevatorRight.set(0.0);
                    elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                    elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                }
            }
        );
    }

    public Command elevatorUpManual() {
        return Commands.run(
            () -> {
                if (true) {
                    elevatorLeft.set(-elevatorUpDownSpeed);
                    elevatorRight.set(elevatorUpDownSpeed);
                } else {
                    elevatorLeft.set(0.0);
                    elevatorRight.set(0.0);
                    elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                    elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                }
            }
        );
    }

    public Command elevatorDownManual() {
        return Commands.run(
            () -> {
                if (true) {
                    elevatorLeft.set(elevatorUpDownSpeed);
                    elevatorRight.set(-elevatorUpDownSpeed);
                } else {
                    elevatorLeft.set(0.0);
                    elevatorRight.set(0.0);
                    elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                    elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                }
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
                    elevatorLeft.set(0.0);
                    elevatorRight.set(0.0);
                    elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                    elevatorRight.setNeutralMode(NeutralModeValue.Brake);
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
                    elevatorLeft.set(0.0);
                    elevatorRight.set(0.0);
                    elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                    elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                }
            }
        );
    }

    public Command stopElevator() {
        return Commands.run(
            () -> {
                elevatorLeft.set(0.0);
                elevatorRight.set(0.0);
            }
        );
    }

    public Command elevatorTestDown() {
        return Commands.run(
            () -> {
                if (Math.abs(elevatorLeft.getPosition().getValueAsDouble()) >= 5.0) {
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
                if (Math.abs(elevatorLeft.getPosition().getValueAsDouble()) <= 55) {
                    CoralSubsystem.flywheelMotor.set(-0.2);
                } else {
                    CoralSubsystem.flywheelMotor.set(0.0);
                }
            }
        );
    }
}
