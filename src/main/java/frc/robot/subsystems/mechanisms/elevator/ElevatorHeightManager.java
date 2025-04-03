package frc.robot.subsystems.mechanisms.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ElevatorHeightManager extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    
    private double targetHeight;
    private boolean isFinishedToggle = false;

    // Hard limits for the elevator


    // How close we have to be to "done"
    private static final double threshold = 2.0;
    
    public ElevatorHeightManager(double desiredHeight, ElevatorSubsystem elevatorSubsystem) {
        // Clamp the incoming desiredHeight between 0 and 65
        this.targetHeight = Math.max(elevatorSubsystem.minHeight, Math.min(elevatorSubsystem.maxHeight, desiredHeight));
        this.elevatorSubsystem = elevatorSubsystem;
        
        addRequirements(elevatorSubsystem);
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
        double currentHeight = -elevatorSubsystem.getElevatorHeight();
        double heightLimiter = 1;//(currentHeight >= -5 ? 0.4: currentHeight >= -10 ? 0.6: currentHeight >= -15 ? 0.8: 1.0);
        // 1. If we’re close enough to target, just hold and finish
        if (Math.abs(currentHeight - targetHeight) <= threshold) {
            System.out.println("Stopping at target");
            
            elevatorSubsystem.elevatorLeft.set(-0.015);
            elevatorSubsystem.elevatorRight.set(0.015);

            isFinishedToggle = true;
        }
        // 2. If we're already above the max limit
        else if (currentHeight >= elevatorSubsystem.maxHeight) {
            if (targetHeight < currentHeight) {
                // We still want to go down
                System.out.println("At/above max limit. Moving down.");
                elevatorSubsystem.elevatorLeft.set(elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
                elevatorSubsystem.elevatorRight.set(-elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
            } else {
                // We want to go up or hold, but we’re at max, so just hold
                System.out.println("At/above max limit. Stopping.");
                elevatorSubsystem.elevatorLeft.set(-0.015);
                elevatorSubsystem.elevatorRight.set(0.015);
                isFinishedToggle = true;
            }
        }
        // 3. If we're below the min limit
        else if (currentHeight <= elevatorSubsystem.minHeight) {
            if (targetHeight > currentHeight) {
                // We want to go up
                System.out.println("At/below min limit. Moving up.");
                elevatorSubsystem.elevatorLeft.set(-elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
                elevatorSubsystem.elevatorRight.set(elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
            } else {
                // We want to go down or hold, but we’re at min, so just hold
                System.out.println("At/below min limit. Stopping.");
                elevatorSubsystem.elevatorLeft.set(-0.015);
                elevatorSubsystem.elevatorRight.set(0.015);
                isFinishedToggle = true;
            }
        }
        // 4. If we’re within normal operating range but above target, move down
        else if (currentHeight > targetHeight) {
            System.out.println("Moving down.");
            elevatorSubsystem.elevatorLeft.set(elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
            elevatorSubsystem.elevatorRight.set(-elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
        }
        // 5. If we’re within normal operating range but below target, move up
        else if (currentHeight < targetHeight) {
            System.out.println("Moving up.");
            elevatorSubsystem.elevatorLeft.set(-elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
            elevatorSubsystem.elevatorRight.set(elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
        }
    }

    @Override
    public boolean isFinished() {
        return isFinishedToggle;
    }

    @Override
    public void end(boolean interrupted) {
        // You might want a small negative feed or 0, depending on your mechanism
        elevatorSubsystem.elevatorLeft.set(-0.015);
        elevatorSubsystem.elevatorRight.set(0.015);
        System.out.println("Command Ended. Motors Stopped.");

        CommandScheduler.getInstance().cancelAll();
    }
}
