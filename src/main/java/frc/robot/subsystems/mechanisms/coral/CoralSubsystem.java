package frc.robot.subsystems.mechanisms.coral;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

    public double angleUpperLimit = 0.668;
    public double angleLowerLimit = 0.3;

    public double krakenAngleUpperLimit = 0.0;
    public double krakenAngleLowerLimit = 7.5;

    public TalonFX angleMotor = new TalonFX(12);
    public TalonFX flywheelMotor = new TalonFX(11);
    public DutyCycleEncoder angleDCEncoder = new DutyCycleEncoder(4);
    
    public Double angleSpeed = 0.1;
    public Double flywheelInSpeed = 0.4;
    public Double flywheelOutSpeed = 0.6;
    
    public Double angleHoldSpeed = 0.015;

    public Command flywheelOut() {
        return Commands.run(
            () -> {
        flywheelMotor.set(flywheelOutSpeed);
    });}
    public Command flywheelIn() {
        return Commands.run(
            () -> {
        flywheelMotor.set(-flywheelInSpeed);
    });}
    public Command flywheelStop() {
        return Commands.run(
            () -> {
        flywheelMotor.set(0.0);
    }); }

    //Angle commands
    public Command angleUp() {
        return Commands.run(
            () -> {
        if (getCoralAngle() != 1 ? (getCoralAngle() < angleUpperLimit): (krakenGetCoralAngle() > krakenAngleUpperLimit)) {
            angleMotor.set(-angleSpeed); }
        else {
            angleMotor.set(0.0);
            angleMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }

    public Command angleDown() {
        return Commands.run(
            () -> {
        if (getCoralAngle() != 1 ? (getCoralAngle() > angleLowerLimit): (krakenGetCoralAngle() < krakenAngleLowerLimit)) {
            angleMotor.set(angleSpeed);} 
        else {
            angleMotor.set(0.0);
            angleMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }


    public Command angleUpSlow() {
        return Commands.run(
            () -> {
                if (getCoralAngle() != 1 ? (getCoralAngle() < angleUpperLimit): (krakenGetCoralAngle() > krakenAngleUpperLimit)) {
        // if (getCoralAngle() < angleUpperLimit) {
            angleMotor.set(-0.1); }
        else {
            angleMotor.set(0.0);
            angleMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }

    public Command angleDownSlow() {
        return Commands.run(
            () -> {

                if (getCoralAngle() != 1 ? (getCoralAngle() > angleLowerLimit): (krakenGetCoralAngle() < krakenAngleLowerLimit)) {
        // if (getCoralAngle() > angleLowerLimit) {
        angleMotor.set(0.1);} 
        else {
            angleMotor.set(0.0);
            angleMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }


    public double getCoralAngle() {
        return angleDCEncoder.get();
    }

    public double krakenGetCoralAngle() {
        return angleMotor.getPosition().getValueAsDouble();
    }
}
