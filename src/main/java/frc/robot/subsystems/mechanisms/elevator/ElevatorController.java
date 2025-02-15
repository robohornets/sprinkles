package frc.robot.subsystems.mechanisms.elevator;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ElevatorController {
    // Moves the elevator up
    public void elevatorUp() {
        ElevatorVariables.elevatorLeft.set(ElevatorVariables.elevatorUpDownSpeed);
        ElevatorVariables.elevatorRight.set(-ElevatorVariables.elevatorUpDownSpeed);
    }

    // Moves the elevator down
    public void elevatorDown() {
        ElevatorVariables.elevatorLeft.set(-ElevatorVariables.elevatorUpDownSpeed);
        ElevatorVariables.elevatorRight.set(ElevatorVariables.elevatorUpDownSpeed);
    }

    // Stops the elevator
    public void stopElevator() {
        ElevatorVariables.elevatorLeft.set(0.0);
        ElevatorVariables.elevatorRight.set(0.0);
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
