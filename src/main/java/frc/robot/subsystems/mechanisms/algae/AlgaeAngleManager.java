package frc.robot.subsystems.mechanisms.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;

public class AlgaeAngleManager extends Command{
    private double angle;
    private AlgaeSubsystem algaeSubsystem;
    private boolean isFinishedToggle = false;
    private double oopsieThreshold = 0.01;

    public AlgaeAngleManager(double angle, AlgaeSubsystem algaeSubsystem) {
        this.angle = Math.max(algaeSubsystem.angleLowerLimit, Math.min(algaeSubsystem.angleUpperLimit, angle));
        this.algaeSubsystem = algaeSubsystem;
        addRequirements(this.algaeSubsystem);
    }

    @Override
    public void initialize() {
        isFinishedToggle = false;
        updateMotorSpeed();
    }

    @Override
    public void execute() {
        updateMotorSpeed();
    }

    private void updateMotorSpeed() {
        double currentAlgaeAngle = getAlgaeAngle();

        if (Math.abs(currentAlgaeAngle - angle) <= oopsieThreshold) {
            System.out.println("Stopping at target");
            algaeSubsystem.angleMotor.set(0.015);
            isFinishedToggle = true;
        } 
        else if (currentAlgaeAngle >= algaeSubsystem.angleUpperLimit) {
            if (angle < currentAlgaeAngle) {
                System.out.println("Above upper limit, moving down");
                algaeSubsystem.angleMotor.set(algaeSubsystem.angleAlgaeSpeed);
            } else {
                System.out.println("Above upper limit, stopping");
                isFinishedToggle = true;
            }
        } 
        else if (currentAlgaeAngle <= algaeSubsystem.angleLowerLimit) {
            if (angle > currentAlgaeAngle) {
                System.out.println("Below lower limit, moving up");
                algaeSubsystem.angleMotor.set(-algaeSubsystem.angleAlgaeSpeed);
            } else {
                System.out.println("Below lower limit, stopping");
                isFinishedToggle = true;
            }
        } 
        else if (currentAlgaeAngle > angle) {
            System.out.println("Moving Down");
            algaeSubsystem.angleMotor.set(algaeSubsystem.angleAlgaeSpeed);
        } 
        else if (currentAlgaeAngle < angle) {
            System.out.println("Moving Up");
            algaeSubsystem.angleMotor.set(-algaeSubsystem.angleAlgaeSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return isFinishedToggle;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Command Ended. Motor Stopped.");
    }

    public double getAlgaeAngle() {
        return algaeSubsystem.angleMotor.getPosition().getValueAsDouble();
    }
}

    

