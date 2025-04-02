
package frc.robot.subsystems.mechanisms.coral.CommandManagers.InOutCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;

public class CoralInCommand extends Command {
    private CoralSubsystem coralSubsystem;
    private boolean isFinishedToggle = false;

    // public double angleUpperLimit = 0.668; // Update: 0.668
    // public double angleLowerLimit = 0.255; // Update: 0.25

    public CoralInCommand(CoralSubsystem coralSubsystem) {
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
        if (coralSubsystem.coralForwardSensor.getDistance(true).getValueAsDouble() < 0.1) {
            coralSubsystem.flywheelMotor.set(0.0);
            isFinishedToggle = true;
        }
        else {
            coralSubsystem.flywheelMotor.set(-0.2);
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
}
