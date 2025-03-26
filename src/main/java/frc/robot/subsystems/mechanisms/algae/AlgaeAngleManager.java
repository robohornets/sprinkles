package frc.robot.subsystems.mechanisms.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;

public class AlgaeAngleManager extends Command{
    private double angle;
    private AlgaeSubsystem algaeSubsystem;
    private boolean isFinishedToggle = false;
    private double oopsieThreshold = 0.01;

    public double angleAlgaeUpperLimit = 0.0;
    public double angleAlgaeLowerLimit = -15.0;

    public AlgaeAngleManager(double angle, AlgaeSubsystem algaeSubsystem) {
        this.angle = Math.max(angleAlgaeLowerLimit, Math.min(angleAlgaeUpperLimit, angle));
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
        else if (currentAlgaeAngle >= angleAlgaeUpperLimit) {
            if (angle < currentAlgaeAngle) {
                System.out.println("Above upper limit, moving down");
                algaeSubsystem.angleMotor.set(algaeSubsystem.angleAlgaeSpeed);
            } else {
                System.out.println("Above upper limit, stopping");
                algaeSubsystem.angleMotor.set(0.015);
                isFinishedToggle = true;
            }
        } 
        else if (currentAlgaeAngle <= angleAlgaeLowerLimit) {
            if (angle > currentAlgaeAngle) {
                System.out.println("Below lower limit, moving up");
                algaeSubsystem.angleMotor.set(-algaeSubsystem.angleAlgaeSpeed);
            } else {
                System.out.println("Below lower limit, stopping");
                algaeSubsystem.angleMotor.set(0.015);
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
        algaeSubsystem.angleMotor.set(-0.015);
        System.out.println("Command Ended. Motor Stopped.");
    }

    public double getAlgaeAngle() {
        return algaeSubsystem.angleMotor.getPosition().getValueAsDouble();
    }
}

    

