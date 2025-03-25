package frc.robot.subsystems.mechanisms.algae;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
    // public void initDefaultCommand() {
    //     setDefaultCommand(Commands.run(
    //         () -> {
    //             angleAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
    //         }
    //     ));
    // }

    public static double angleAlgaeUpperLimit = 0.0;
    public static double angleAlgaeLowerLimit = -15.0;

    public static TalonFX angleAlgaeMotor = new TalonFX(15);
    public static TalonFX flywheelAlgaeMotor = new TalonFX(14);
    public static DutyCycleEncoder angleAlgaeDCEncoder = new DutyCycleEncoder(3);

    public static Boolean angleAlgaeDisabled = false;
    public static Boolean flywheelAlgaeDisabled = false;
    
    // Angle 12, flywheel 11
    public static Double angleAlgaeSpeed = 0.2;
    public static Double flywheelAlgaeInSpeed = 0.3;
    public static Double flywheelAlgaeOutSpeed = 1.0;

    public static Double angleAlgaeHoldSpeed = 0.015;

            //Flywheel commands
    public Command flywheelAlgaeOut() {
        return Commands.run(
            () -> {
                flywheelAlgaeMotor.set(flywheelAlgaeOutSpeed);
    });}
    
    public Command flywheelAlgaeIn() {
        return Commands.run(
            () -> {
        flywheelAlgaeMotor.set(-flywheelAlgaeInSpeed);
    });}

    public Command flywheelAlgaeStop() {
        return Commands.run(
            () -> {
        flywheelAlgaeMotor.set(0.0);
    }); }

    //Angle commands
    public Command angleAlgaeUp() {
        return Commands.run(
            () -> {
        if (getAlgaeAngle() < angleAlgaeUpperLimit) {
            angleAlgaeMotor.set(angleAlgaeSpeed); }
        else {
            angleAlgaeMotor.set(0.0);
            angleAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }

    public Command angleAlgaeDown() {
        return Commands.run(
            () -> {
        if (getAlgaeAngle() > angleAlgaeLowerLimit) {
            angleAlgaeMotor.set(-angleAlgaeSpeed);} 
        else {
            angleAlgaeMotor.set(0.0);
            angleAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }


    public Command angleAlgaeUpSlow() {
        return Commands.run(
            () -> {
        if (getAlgaeAngle() < angleAlgaeUpperLimit) {
            angleAlgaeMotor.set(-0.1); }
        else {
            angleAlgaeMotor.set(0.0);
            angleAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }

    public Command angleAlgaeDownSlow() {
        return Commands.run(
            () -> {
        if (getAlgaeAngle() > angleAlgaeLowerLimit) {
        angleAlgaeMotor.set(0.1);} 
        else {
            angleAlgaeMotor.set(0.0);
            angleAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
            }
    }); }


    public static double getAlgaeAngle() {
        return angleAlgaeMotor.getPosition().getValueAsDouble();
        //return angleAlgaeDCEncoder.get();
    }
}
