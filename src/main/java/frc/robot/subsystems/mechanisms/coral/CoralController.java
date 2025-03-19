package frc.robot.subsystems.mechanisms.coral;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CoralController {

    //Flywheel commands
    public Command flywheelOut() {
        return Commands.run(
            () -> {
        CoralSubsystem.flywheelMotor.set(CoralSubsystem.flywheelOutSpeed);
    });}
    public Command flywheelIn() {
        return Commands.run(
            () -> {
        CoralSubsystem.flywheelMotor.set(-CoralSubsystem.flywheelInSpeed);
    });}
    public Command flywheelStop() {
        return Commands.run(
            () -> {
        CoralSubsystem.flywheelMotor.set(0.0);
    }); }

    //Angle commands
    public Command angleUp() {
        return Commands.run(
            () -> {
        if (getCoralAngle() < CoralSubsystem.angleUpperLimit) {
            CoralSubsystem.angleMotor.set(-CoralSubsystem.angleSpeed); }
        else {
            CoralSubsystem.angleMotor.set(0.0);
            CoralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }

    public Command angleDown() {
        return Commands.run(
            () -> {
        if (getCoralAngle() > CoralSubsystem.angleLowerLimit) {
            CoralSubsystem.angleMotor.set(CoralSubsystem.angleSpeed);} 
        else {
            CoralSubsystem.angleMotor.set(0.0);
            CoralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }


    public Command angleUpSlow() {
        return Commands.run(
            () -> {
        if (getCoralAngle() < CoralSubsystem.angleUpperLimit) {
            CoralSubsystem.angleMotor.set(-0.1); }
        else {
            CoralSubsystem.angleMotor.set(0.0);
            CoralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }

    public Command angleDownSlow() {
        return Commands.run(
            () -> {
        if (getCoralAngle() > CoralSubsystem.angleLowerLimit) {
        CoralSubsystem.angleMotor.set(0.1);} 
        else {
            CoralSubsystem.angleMotor.set(0.0);
            CoralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }


    public static double getCoralAngle() {
        return CoralSubsystem.angleDCEncoder.get();
    }
}