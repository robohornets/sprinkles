package frc.robot.subsystems.mechanisms.elevator;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ElevatorController {
    public final TalonFX elevatorLeft = new TalonFX(9);
    public final TalonFX elevatorRight = new TalonFX(10);

    // Disables all control of the elevator
    public static Boolean elevatorDownDisabled = false;
    public static Boolean elevatorUpDisabled = false;

    // CANrange sensor and trigger for lower elevator
    public CANrange elevatorDownSensor = new CANrange(34);
    Trigger elevatorDownTrigger = new Trigger(() -> elevatorDownSensor.getDistance(true).refresh().getValueAsDouble() < 0.2);

    public Command elevatorUp() {
        return Commands.run(
            () -> {
                elevatorLeft.set(ElevatorVariables.elevatorUpDownSpeed);
                elevatorRight.set(-ElevatorVariables.elevatorUpDownSpeed);
            }
        );
    }

    public Command elevatorDown() {
        return Commands.run(
            () -> {
        elevatorLeft.set(-ElevatorVariables.elevatorUpDownSpeed);
        elevatorRight.set(ElevatorVariables.elevatorUpDownSpeed);
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

    // These manage the enabled/disabled state of the elevator's range of motion
    public void disableElevatorUp() {
        ElevatorVariables.elevatorUpDisabled = true;
        ElevatorVariables.elevatorDownDisabled = false;
    }
    
    public void enableElevatorUp() {
        ElevatorVariables.elevatorUpDisabled = false;
    }
    public void disableElevatorDown() {
        ElevatorVariables.elevatorDownDisabled = true;
        ElevatorVariables.elevatorUpDisabled = false;
    }
    public void enableElevatorDown() {
        ElevatorVariables.elevatorDownDisabled = false;
    }

    public double getHeight() {
        return 0.0;
    }
}
