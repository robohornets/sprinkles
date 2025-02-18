package frc.robot.subsystems.mechanisms.coral;

import edu.wpi.first.wpilibj2.command.Commands;

public class CoralController {

    //Flywheel commands
    public void flywheelOut() {
        Commands.run(
            () -> {
        CoralVariables.flywheelMotor.set(CoralVariables.flywheelSpeed);
    });}
    public void flywheelIn() {
        Commands.run(
            () -> {
        CoralVariables.flywheelMotor.set(-CoralVariables.flywheelSpeed);
    });}
    public void flywheelStop() {
        Commands.run(
            () -> {
        CoralVariables.flywheelMotor.set(0.0);
    }); }

    //Angle commands
    public void angleUp() {
        Commands.run(
            () -> {
        CoralVariables.angleMotor.set(CoralVariables.angleSpeed);
    }); }
    public void angleDown() {
        Commands.run(
            () -> {
        CoralVariables.angleMotor.set(-CoralVariables.angleSpeed);
    }); }
    public void angleStop() {
        Commands.run(
            () -> {
        CoralVariables.angleMotor.set(0.0);
    }); } 
    public void angleGetAngle() {
        Commands.run(
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

