
package frc.robot.subsystems.mechanisms.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CoralPositionManager extends Command {
    private CoralSubsystem coralSubsystem;
    private boolean isFinishedToggle = false;

    // public double angleUpperLimit = 0.668; // Update: 0.668
    // public double angleLowerLimit = 0.255; // Update: 0.25

    public CoralPositionManager(CoralSubsystem coralSubsystem) {
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
        if (coralSubsystem.coralForwardTrigger.getAsBoolean()) {
            coralSubsystem.flywheelMotor.set(0.0);
            isFinishedToggle = true;
        }
        else {
            coralSubsystem.flywheelMotor.set(0.2);
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
