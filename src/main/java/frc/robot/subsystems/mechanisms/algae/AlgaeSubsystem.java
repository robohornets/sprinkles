package frc.robot.subsystems.mechanisms.algae;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
    public double angleAlgaeUpperLimit = 0.0;
    public double angleAlgaeLowerLimit = -15.0;

    public TalonFX angleMotor = new TalonFX(15, "Mechanisms");
    public DutyCycleEncoder angleDCEncoder = new DutyCycleEncoder(3);
    
    public Double angleAlgaeSpeed = 0.2;
    public Double flywheelAlgaeInSpeed = 0.3;
    public Double flywheelAlgaeOutSpeed = 1.0;

    public Double angleAlgaeHoldSpeed = 0.015;

    //Angle commands
    public Command angleAlgaeUp() {
        return Commands.run(
            () -> {
                if (getAlgaeAngle() < angleAlgaeUpperLimit) {
                    angleMotor.set(angleAlgaeSpeed); }
                else {
                    angleMotor.set(0.0);
                }
            }
        );
    }

    public Command angleAlgaeDown() {
        return Commands.run(
            () -> {
                if (getAlgaeAngle() > angleAlgaeLowerLimit) {
                    angleMotor.set(-angleAlgaeSpeed);} 
                else {
                    angleMotor.set(0.0);
                }
            }
        );
    }


    public Command angleAlgaeUpSlow() {
        return Commands.run(
            () -> {
                if (getAlgaeAngle() < angleAlgaeUpperLimit) {
                    angleMotor.set(-0.1); }
                else {
                    angleMotor.set(0.0);
                }
            }
        );
    }

    public Command angleAlgaeDownSlow() {
        return Commands.run(
            () -> {
                if (getAlgaeAngle() > angleAlgaeLowerLimit) {
                angleMotor.set(0.1);} 
                else {
                    angleMotor.set(0.0);
                }
            }
        );
    }


    public double getAlgaeAngle() {
        return angleMotor.getPosition().getValueAsDouble();
    }
}
