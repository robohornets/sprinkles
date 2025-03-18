package frc.robot.subsystems.mechanisms.algae;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
    public static TalonFX angleAlgaeMotor = new TalonFX(15);
    public static TalonFX flywheelAlgaeMotor = new TalonFX(14);
    public static DutyCycleEncoder angleAlgaeDCEncoder = new DutyCycleEncoder(3);

    public static Boolean angleAlgaeDisabled = false;
    public static Boolean flywheelAlgaeDisabled = false;
    
    // Angle 12, flywheel 11
    public static Double angleAlgaeSpeed = 0.2;
    public static Double flywheelAlgaeInSpeed = 0.3;
    public static Double flywheelAlgaeOutSpeed = 1.0;

    public static Double angleAlgaeHoldSpeed = 0.015;
}
