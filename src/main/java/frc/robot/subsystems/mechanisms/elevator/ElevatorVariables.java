package frc.robot.subsystems.mechanisms.elevator;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ElevatorVariables {
    public static Double elevatorUpDownSpeed = 0.2;

    public static TalonFX elevatorLeft = new TalonFX(9);
    public static TalonFX elevatorRight = new TalonFX(10);

    // Disables all control of the elevator
    public static Boolean elevatorDownDisabled = false;
    public static Boolean elevatorUpDisabled = false;

    // CANrange sensor and trigger for lower elevator
    public CANrange elevatorDownSensor = new CANrange(34);
    Trigger elevatorDownTrigger = new Trigger(() -> elevatorDownSensor.getDistance(true).refresh().getValueAsDouble() < 0.2);

}
