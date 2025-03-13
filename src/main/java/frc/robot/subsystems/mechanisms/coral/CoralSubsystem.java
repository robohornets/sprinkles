package frc.robot.subsystems.mechanisms.coral;

import edu.wpi.first.wpilibj2.command.Command;

public class CoralSubsystem extends Command {
    private double angle;
    private CoralVariables coralSubsystem;
    private boolean isFinishedToggle = false;
    private double oopsieThreshold = 0.01;

    public double angleUpperLimit = 0.82;
    public double angleLowerLimit = 0.48;

    public CoralSubsystem(double angle, CoralVariables coralSubsystem) {
        this.angle = Math.max(angleLowerLimit, Math.min(angleUpperLimit, angle));
        this.coralSubsystem = coralSubsystem;
        addRequirements(this.coralSubsystem);
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
        double currentAngle = getCoralAngle();

        if (Math.abs(currentAngle - angle) <= oopsieThreshold) {
            System.out.println("Stopping at target");
            coralSubsystem.angleMotor.set(-0.015);
            isFinishedToggle = true;
        } 
        else if (currentAngle >= angleUpperLimit) {
            if (angle < currentAngle) {
                System.out.println("Above upper limit, moving down");
                coralSubsystem.angleMotor.set(coralSubsystem.angleSpeed);
            } else {
                System.out.println("Above upper limit, stopping");
                coralSubsystem.angleMotor.set(-0.015);
                isFinishedToggle = true;
            }
        } 
        else if (currentAngle <= angleLowerLimit) {
            if (angle > currentAngle) {
                System.out.println("Below lower limit, moving up");
                coralSubsystem.angleMotor.set(-coralSubsystem.angleSpeed);
            } else {
                System.out.println("Below lower limit, stopping");
                coralSubsystem.angleMotor.set(-0.015);
                isFinishedToggle = true;
            }
        } 
        else if (currentAngle > angle) {
            System.out.println("Moving Down");
            coralSubsystem.angleMotor.set(coralSubsystem.angleSpeed);
        } 
        else if (currentAngle < angle) {
            System.out.println("Moving Up");
            coralSubsystem.angleMotor.set(-coralSubsystem.angleSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return isFinishedToggle;
    }

    @Override
    public void end(boolean interrupted) {
        coralSubsystem.angleMotor.set(-0.015);
        System.out.println("Command Ended. Motor Stopped.");
    }

    public static double getCoralAngle() {
        return CoralVariables.angleDCEncoder.get();
    }
}
