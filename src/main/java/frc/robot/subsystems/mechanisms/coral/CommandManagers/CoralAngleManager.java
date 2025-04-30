package frc.robot.subsystems.mechanisms.coral.CommandManagers;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;

public class CoralAngleManager extends Command {
    private double krakenAngle;
    private double pigeonAngle;
    private CoralSubsystem coralSubsystem;
    private boolean isFinishedToggle = false;
    private double pigeonThreshold = 1;

    public CoralAngleManager(double krakenAngle, double pigeonAngle, CoralSubsystem coralSubsystem) {
        this.krakenAngle = krakenAngle;
        this.pigeonAngle = pigeonAngle;
        this.coralSubsystem = coralSubsystem;
        addRequirements(this.coralSubsystem);
    }

    @Override
    public void initialize() {
        isFinishedToggle = false;
        coralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
        //updateMotorSpeed();
    }

    @Override
    public void execute() {
        updateMotorSpeed();
    }

    private void updateMotorSpeed() {
        double currentAngle = pigeonGetCoralAngle();
        double targetAngle = pigeonAngle;
        double lowerLimit = coralSubsystem.pigeonAngleLowerLimit;
        double upperLimit = coralSubsystem.pigeonAngleUpperLimit;

        // Debug print to see what's going on
        System.out.printf("Current: %.2f Target: %.2f (Limit: %.2f - %.2f)\n", currentAngle, targetAngle, lowerLimit, upperLimit);

        if (Math.abs(currentAngle - targetAngle) <= pigeonThreshold) {
            System.out.println("At target. Stopping.");
            coralSubsystem.angleMotor.set(0.0);
            isFinishedToggle = true;
            return;
        }

        if (currentAngle > targetAngle) {
            if (currentAngle <= lowerLimit) {
                System.out.println("At/below lower limit. Cannot move down.");
                coralSubsystem.angleMotor.set(0.0);
                return;
            }
            System.out.println("Moving Down.");
            coralSubsystem.angleMotor.set(-coralSubsystem.angleSpeed);
        } else {
            if (currentAngle >= upperLimit) {
                System.out.println("At/above upper limit. Cannot move up.");
                coralSubsystem.angleMotor.set(0.0);
                return;
            }
            System.out.println("Moving Up.");
            coralSubsystem.angleMotor.set(coralSubsystem.angleSpeed);
        }
    }
    

    @Override
    public boolean isFinished() {
        return isFinishedToggle;
    }

    @Override
    public void end(boolean interrupted) {
        coralSubsystem.angleMotor.set(coralSubsystem.angleHoldSpeed);
        System.out.println("Command Ended. Motor Stopped.");
    }
    public double pigeonGetCoralAngle() {
        return coralSubsystem.pigeonGetCoralAngle();
    }
}
