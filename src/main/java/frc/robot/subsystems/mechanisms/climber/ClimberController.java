package frc.robot.subsystems.mechanisms.climber;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;

public class ClimberController {
    public final TalonFX climberLeft = new TalonFX(11);
    public final TalonFX climberRight = new TalonFX(12);

    // CANrange sensor and trigger for lower elevator
    public CANrange elevatorDownSensor = new CANrange(35);
    Trigger elevatorDownTrigger = new Trigger(() -> 
        elevatorDownSensor.getDistance(true).refresh().getValueAsDouble() < 0.2
    );

    public void elevatorUp() {
        climberLeft.set(ElevatorSubsystem.elevatorUpDownSpeed);
        climberRight.set(-ElevatorSubsystem.elevatorUpDownSpeed);
    }

    public void elevatorDown() {
        climberLeft.set(-ClimberVariables.climberUpDownSpeed);
        climberRight.set(ClimberVariables.climberUpDownSpeed);
    }

    public void stopElevator() {
        climberLeft.set(0.0);
        climberRight.set(0.0);
    }
}
