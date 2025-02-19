package frc.robot.subsystems.mechanisms.coral;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

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
        CoralVariables.angleMotor.set(CoralVariables.angleSpeed);
    }); }
    public Command angleDown() {
        return Commands.run(
            () -> {
        CoralVariables.angleMotor.set(-CoralVariables.angleSpeed);
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

