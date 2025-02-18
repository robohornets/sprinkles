package frc.robot.subsystems.mechanisms.coral;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class CoralVariables {
    public static TalonFX angleMotor = new TalonFX(12);
    public static TalonFX flywheelMotor = new TalonFX(11);
    public static DutyCycleEncoder angleDCEncoder = new DutyCycleEncoder(2);

    public static Boolean angleDisabled = false;
    public static Boolean flywheelDisabled = false;
    
    // Angle 12, flywheel 11
    public static Double angleSpeed = 0.2;
    public static Double flywheelSpeed = 0.2;

}
