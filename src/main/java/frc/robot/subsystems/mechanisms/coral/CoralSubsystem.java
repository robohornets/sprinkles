package frc.robot.subsystems.mechanisms.coral;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

    public static double angleUpperLimit = 0.668;
    public static double angleLowerLimit = 0.3;

    public static double krakenAngleUpperLimit = 0.0;
    public static double krakenAngleLowerLimit = 7.5;

    public static TalonFX angleMotor = new TalonFX(12);
    public static TalonFX flywheelMotor = new TalonFX(11);
    public static DutyCycleEncoder angleDCEncoder = new DutyCycleEncoder(4);

    public static Boolean angleDisabled = false;
    public static Boolean flywheelDisabled = false;
    
    public static Double angleSpeed = 0.1;
    public static Double flywheelInSpeed = 0.4;
    public static Double flywheelOutSpeed = 0.6;
    
    public static Double angleHoldSpeed = 0.015;

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


    public static double getCoralAngle() {
        return angleDCEncoder.get();
    }

    public static double krakenGetCoralAngle() {
        return angleMotor.getPosition().getValueAsDouble();
    }
}
