
package frc.robot.subsystems.mechanisms.coral.CommandManagers.InOutCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;

public class FunnelOutCommand extends Command {
    private CoralSubsystem coralSubsystem;
    private boolean isFinishedToggle = false;

    public FunnelOutCommand(CoralSubsystem coralSubsystem) {
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
        if (coralSubsystem.funnelSensor.getDistance(true).getValueAsDouble() > 0.1) {
            coralSubsystem.funnelLeft.set(0.0);
            coralSubsystem.funnelRight.set(0.0);
            isFinishedToggle = true;
        }
        else {
            coralSubsystem.funnelLeft.set(-coralSubsystem.funnelSpeed);
            coralSubsystem.funnelRight.set(coralSubsystem.funnelSpeed);
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
