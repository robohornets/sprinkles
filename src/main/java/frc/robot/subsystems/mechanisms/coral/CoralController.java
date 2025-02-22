package frc.robot.subsystems.mechanisms.coral;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CoralController {

    //Flywheel commands
    public Command flywheelOut() {
        return Commands.run(
            () -> {
        CoralVariables.flywheelMotor.set(CoralVariables.flywheelSpeed);
    });}
    public Command flywheelIn() {
        return Commands.run(
            () -> {
        CoralVariables.flywheelMotor.set(-CoralVariables.flywheelSpeed);
    });}
    public Command flywheelStop() {
        return Commands.run(
            () -> {
        CoralVariables.flywheelMotor.set(0.0);
    }); }

    //Angle commands
    public Command angleUp() {
        
        return Commands.run(
            () -> {
        if (CoralVariables.angleMotor.getPosition().getValueAsDouble() >= -0.1) {
            CoralVariables.angleMotor.set(-CoralVariables.angleSpeed); }
        else {
            CoralVariables.angleMotor.set(0.0);
            CoralVariables.angleMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }
    public Command angleDown() {
        return Commands.run(
            () -> {
        if (Math.abs(CoralVariables.angleMotor.getPosition().getValueAsDouble()) <= 8.2) {
        CoralVariables.angleMotor.set(CoralVariables.angleSpeed);} 
        else {
            CoralVariables.angleMotor.set(0.0);
            CoralVariables.angleMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }
    public Command angleStop() {
        return Commands.run(
            () -> {
        CoralVariables.angleMotor.set(0.0);
    }); } 
    public Command angleGetAngle() {
        return Commands.run(
            () -> {
        CoralVariables.angleDCEncoder.get();
    }); } 

    //Enabled/Disabled for angle
    public void enableAngle() {
        Commands.run(
            () -> {
        CoralVariables.angleDisabled = false;
    }); }
    public void disableAngle() {
        Commands.run(
            () -> {
        CoralVariables.angleDisabled = true;
    }); }
}