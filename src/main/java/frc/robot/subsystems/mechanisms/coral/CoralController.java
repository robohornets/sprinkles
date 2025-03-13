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
        if (true/*getCoralAngle() > 0.5*/) {
            CoralVariables.angleMotor.set(-CoralVariables.angleSpeed); }
        else {
            CoralVariables.angleMotor.set(0.0);
            CoralVariables.angleMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }

    public Command angleDown() {
        return Commands.run(
            () -> {
        if (true/*getCoralAngle() < 0.3*/) {
        CoralVariables.angleMotor.set(CoralVariables.angleSpeed);} 
        else {
            CoralVariables.angleMotor.set(0.0);
            CoralVariables.angleMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }

    public double getCoralAngle() {
        return CoralVariables.angleDCEncoder.get();
    }
}