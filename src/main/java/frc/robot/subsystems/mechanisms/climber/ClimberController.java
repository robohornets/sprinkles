package frc.robot.subsystems.mechanisms.climber;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;

public class ClimberController {
    public void elevatorUp() {
        ClimberVariables.alexHonnold.set(0.5);
    }

    public void elevatorDown() {
        ClimberVariables.alexHonnold.set(-0.5);
    }

    public void stopElevator() {
        ClimberVariables.alexHonnold.set(0.0);
    }
}
