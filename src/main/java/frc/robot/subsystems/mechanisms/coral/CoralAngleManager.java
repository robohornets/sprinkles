package frc.robot.subsystems.mechanisms.coral;

import edu.wpi.first.wpilibj2.command.Command;

public class CoralAngleManager extends Command {
    private double angle;
    private double krakenAngle;
    private CoralSubsystem coralSubsystem;
    private boolean isFinishedToggle = false;
    private double oopsieThreshold = 0.01;
    private double krakenOopsieThreshold = 0.1;

    // public double angleUpperLimit = 0.668; // Update: 0.668
    // public double angleLowerLimit = 0.255; // Update: 0.255

    public CoralAngleManager(double angle, double krakenAngle, CoralSubsystem coralSubsystem) {
        this.angle = Math.max(coralSubsystem.angleLowerLimit, Math.min(coralSubsystem.angleUpperLimit, angle));
        this.krakenAngle = krakenAngle;
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
        double krakenCurrentAngle = krakenGetCoralAngle();

        if (currentAngle != 1 ? (Math.abs(currentAngle - angle) <= oopsieThreshold): (Math.abs(krakenCurrentAngle - krakenAngle) <= krakenOopsieThreshold)) {
            System.out.println("Stopping at target");
            //coralSubsystem.angleMotor.set(-0.015);
            isFinishedToggle = true;
        } 
        else if (currentAngle != 1 ? (currentAngle >= coralSubsystem.angleUpperLimit) : (krakenCurrentAngle <= coralSubsystem.krakenAngleUpperLimit)) {
            if (currentAngle != 1 ? (angle < currentAngle) : (krakenAngle > krakenCurrentAngle)) {
                System.out.println("Above upper limit, moving down");
                coralSubsystem.angleMotor.set(coralSubsystem.angleSpeed);
            } else {
                System.out.println("Above upper limit, stopping");
                // coralSubsystem.angleMotor.set(-0.015);
                isFinishedToggle = true;
            }
        } 
        else if (currentAngle != 1 ? (currentAngle <= coralSubsystem.angleLowerLimit) : (krakenCurrentAngle >= coralSubsystem.krakenAngleLowerLimit)) {
            if (currentAngle != 1 ? (angle > currentAngle) : (krakenAngle < krakenCurrentAngle)) {
                System.out.println("Below lower limit, moving up");
                coralSubsystem.angleMotor.set(-coralSubsystem.angleSpeed);
            } else {
                System.out.println("Below lower limit, stopping");
                // coralSubsystem.angleMotor.set(-0.015);
                isFinishedToggle = true;
            }
        } 
        else if (currentAngle != 1 ? (currentAngle > angle) : (krakenCurrentAngle < krakenAngle)) {
            System.out.println("Moving Down");
            coralSubsystem.angleMotor.set(coralSubsystem.angleSpeed);
        } 
        else if (currentAngle != 1 ? (currentAngle < angle) : (krakenCurrentAngle > krakenAngle)) {
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

    public double getCoralAngle() {
        return coralSubsystem.angleDCEncoder.get();
    }

    public double krakenGetCoralAngle() {
        return coralSubsystem.angleMotor.getPosition().getValueAsDouble();
    }
}
