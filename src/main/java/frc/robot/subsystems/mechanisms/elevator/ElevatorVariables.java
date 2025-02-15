package frc.robot.subsystems.mechanisms.elevator;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ElevatorVariables {
    public static final double elevatorUpDownSpeed = 0.2;

    public static final TalonFX elevatorLeft = new TalonFX(9);
    public static final TalonFX elevatorRight = new TalonFX(10);

    // Disables all control of the elevator
    public static boolean elevatorDownDisabled = false;
    public static boolean elevatorUpDisabled = false;

    // CANrange sensor and trigger for lower elevator
    public static final CANrange elevatorDownSensor = new CANrange(34);

    public static Trigger disableDownTrigger = new Trigger(
        () -> ElevatorVariables.elevatorEncoder.getDistance() >= -1 && ElevatorVariables.elevatorEncoder.getDistance() <= 0.5
    );
    
    public static Trigger disableUpTrigger = new Trigger(
        () -> ElevatorVariables.elevatorEncoder.getDistance() > 5
    );

    // **Added Encoder (REV Through Bore in Quadrature Mode)**
    public static final Encoder elevatorEncoder = new Encoder(0, 1);

    // **Zero the encoder at startup**
    static {
        elevatorEncoder.reset();
        System.out.println("Elevator encoder zeroed at startup.");
    }
}
