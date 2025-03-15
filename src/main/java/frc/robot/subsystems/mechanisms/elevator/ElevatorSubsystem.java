package frc.robot.subsystems.mechanisms.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ElevatorSubsystem extends Command {
    private ElevatorVariables elevatorSubsystem;
    
    private double targetHeight;
    private boolean isFinishedToggle = false;

    // Hard limits for the elevator
    private static final double minHeight = 0.0;
    private static final double maxHeight = 65.0;

    // How close we have to be to “done”
    private static final double threshold = 2.0;
    
    public ElevatorSubsystem(double desiredHeight, ElevatorVariables elevatorSubsystem) {
        // Clamp the incoming desiredHeight between 0 and 65
        this.targetHeight = Math.max(minHeight, Math.min(maxHeight, desiredHeight));
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
        double currentHeight = -elevatorSubsystem.elevatorLeft.getPosition().getValueAsDouble();

        // 1. If we’re close enough to target, just hold and finish
        if (Math.abs(currentHeight - targetHeight) <= threshold) {
            System.out.println("Stopping at target");
            
            elevatorSubsystem.elevatorLeft.set(-0.015);
            elevatorSubsystem.elevatorRight.set(0.015);

            elevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
            elevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);

            isFinishedToggle = true;
        }
        // 2. If we're already above the max limit
        else if (currentHeight >= maxHeight) {
            if (targetHeight < currentHeight) {
                // We still want to go down
                System.out.println("At/above max limit. Moving down.");
                elevatorSubsystem.elevatorLeft.set(ElevatorVariables.elevatorUpDownSpeed);
                elevatorSubsystem.elevatorRight.set(-ElevatorVariables.elevatorUpDownSpeed);
            } else {
                // We want to go up or hold, but we’re at max, so just hold
                System.out.println("At/above max limit. Stopping.");
                elevatorSubsystem.elevatorLeft.set(-0.015);
                elevatorSubsystem.elevatorRight.set(0.015);
                isFinishedToggle = true;
            }
        }
        // 3. If we're below the min limit
        else if (currentHeight <= minHeight) {
            if (targetHeight > currentHeight) {
                // We want to go up
                System.out.println("At/below min limit. Moving up.");
                elevatorSubsystem.elevatorLeft.set(-ElevatorVariables.elevatorUpDownSpeed);
                elevatorSubsystem.elevatorRight.set(ElevatorVariables.elevatorUpDownSpeed);
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
            elevatorSubsystem.elevatorLeft.set(ElevatorVariables.elevatorUpDownSpeed);
            elevatorSubsystem.elevatorRight.set(-ElevatorVariables.elevatorUpDownSpeed);
        }
        // 5. If we’re within normal operating range but below target, move up
        else if (currentHeight < targetHeight) {
            System.out.println("Moving up.");
            elevatorSubsystem.elevatorLeft.set(-ElevatorVariables.elevatorUpDownSpeed);
            elevatorSubsystem.elevatorRight.set(ElevatorVariables.elevatorUpDownSpeed);
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
    }
}
