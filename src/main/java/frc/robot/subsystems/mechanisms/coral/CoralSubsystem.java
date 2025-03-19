package frc.robot.subsystems.mechanisms.coral;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

    // public CoralSubsystem() {
    //     setDefaultCommand(Commands.run(() -> {
    //         angleMotor.set(-angleHoldSpeed);
    //     }));
    // }

    public static TalonFX angleMotor = new TalonFX(12);
    public static TalonFX flywheelMotor = new TalonFX(11);
    public static DutyCycleEncoder angleDCEncoder = new DutyCycleEncoder(2);

    public static Boolean angleDisabled = false;
    public static Boolean flywheelDisabled = false;
    
    public static Double angleSpeed = 0.1;
    public static Double flywheelInSpeed = 0.2;
    public static Double flywheelOutSpeed = 0.6;
    
    public static Double angleHoldSpeed = 0.015;
}
