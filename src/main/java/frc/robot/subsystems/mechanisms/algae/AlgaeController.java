package frc.robot.subsystems.mechanisms.algae;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;

public class AlgaeController {
        //Flywheel commands
    public Command flywheelAlgaeOut() {
        return Commands.run(
            () -> {
                AlgaeSubsystem.flywheelAlgaeMotor.set(AlgaeSubsystem.flywheelAlgaeOutSpeed);
    });}
    
    public Command flywheelAlgaeIn() {
        return Commands.run(
            () -> {
        AlgaeSubsystem.flywheelAlgaeMotor.set(-AlgaeSubsystem.flywheelAlgaeInSpeed);
    });}

    public Command flywheelAlgaeStop() {
        return Commands.run(
            () -> {
        AlgaeSubsystem.flywheelAlgaeMotor.set(0.0);
    }); }

    //Angle commands
    public Command angleAlgaeUp() {
        return Commands.run(
            () -> {
        if (getAlgaeAngle() < 0.99) {
            AlgaeSubsystem.angleAlgaeMotor.set(-AlgaeSubsystem.angleAlgaeSpeed); }
        else {
            AlgaeSubsystem.angleAlgaeMotor.set(0.0);
            AlgaeSubsystem.angleAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }

    public Command angleAlgaeDown() {
        return Commands.run(
            () -> {
        if (getAlgaeAngle() > 0.3) {
            AlgaeSubsystem.angleAlgaeMotor.set(AlgaeSubsystem.angleAlgaeSpeed);} 
        else {
            AlgaeSubsystem.angleAlgaeMotor.set(0.0);
            AlgaeSubsystem.angleAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }


    public Command angleAlgaeUpSlow() {
        return Commands.run(
            () -> {
        if (getAlgaeAngle() < 0.99) {
            AlgaeSubsystem.angleAlgaeMotor.set(-0.1); }
        else {
            AlgaeSubsystem.angleAlgaeMotor.set(0.0);
            AlgaeSubsystem.angleAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }

    public Command angleAlgaeDownSlow() {
        return Commands.run(
            () -> {
        if (getAlgaeAngle() > 0.1) {
        AlgaeSubsystem.angleAlgaeMotor.set(0.1);} 
        else {
            AlgaeSubsystem.angleAlgaeMotor.set(0.0);
            AlgaeSubsystem.angleAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }


    public static double getAlgaeAngle() {
        return AlgaeSubsystem.angleAlgaeDCEncoder.get();
    }
}
