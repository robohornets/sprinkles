
package frc.robot.subsystems.mechanisms.coral.CommandManagers;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;

public class HandoffManager extends Command {
    private CoralSubsystem coralSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private boolean isFinishedToggle = false;
    private static final double threshold = 1.0;

    public HandoffManager(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.coralSubsystem = coralSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(this.coralSubsystem, this.elevatorSubsystem);
    }

    private int handoffStep = 1;
    private double handoffStartTime = -1;
    private double downBrakeDelay = 0.25;

    @Override
    public void initialize() {
        isFinishedToggle = false;
        handoffStep = 1;
        handoffStartTime = -1;
        updateState();
    }    

    @Override
    public void execute() {
        updateState();
    }

    private void updateState() {
        // Step 0 intakes in the funnel until the CANrange detects coral
        if (handoffStep == 0) {
            if (coralSubsystem.funnelSensor.getDistance(true).getValueAsDouble() < 0.1) {
                coralSubsystem.funnelLeft.set(0.0);
                coralSubsystem.funnelRight.set(0.0);
                handoffStep++;
            }
            else {
                coralSubsystem.funnelLeft.set(-coralSubsystem.funnelSpeed);
                coralSubsystem.funnelRight.set(coralSubsystem.funnelSpeed);
            }
        }
        // Step 1 moves the elevator to the bottom
        else if (handoffStep == 1) {
            double currentHeight = -elevatorSubsystem.getElevatorHeight();
            double heightLimiter = (currentHeight >= -5 ? 0.4: currentHeight >= -10 ? 0.6: currentHeight >= -15 ? 0.8: 1.0);
            // 1. If we’re close enough to target, just hold and finish
            if (Math.abs(currentHeight - 0) <= threshold) {
                System.out.println("Stopping at target");
                
                elevatorSubsystem.elevatorLeft.set(-0.015);
                elevatorSubsystem.elevatorRight.set(0.015);

                handoffStep++;
            }
            // 2. If we're already above the max limit
            else if (currentHeight >= elevatorSubsystem.maxHeight) {
                if (0 < currentHeight) {
                    // We still want to go down
                    System.out.println("At/above max limit. Moving down.");
                    elevatorSubsystem.elevatorLeft.set(elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
                    elevatorSubsystem.elevatorRight.set(-elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
                } else {
                    // We want to go up or hold, but we’re at max, so just hold
                    System.out.println("At/above max limit. Stopping.");
                    elevatorSubsystem.elevatorLeft.set(-0.015);
                    elevatorSubsystem.elevatorRight.set(0.015);
                    handoffStep++;
                }
            }
            // 3. If we're below the min limit
            else if (currentHeight <= elevatorSubsystem.minHeight) {
                if (0 > currentHeight) {
                    // We want to go up
                    System.out.println("At/below min limit. Moving up.");
                    elevatorSubsystem.elevatorLeft.set(-elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
                    elevatorSubsystem.elevatorRight.set(elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
                } else {
                    // We want to go down or hold, but we’re at min, so just hold
                    System.out.println("At/below min limit. Stopping.");
                    elevatorSubsystem.elevatorLeft.set(-0.015);
                    elevatorSubsystem.elevatorRight.set(0.015);
                    handoffStep++;
                }
            }
            // 4. If we’re within normal operating range but above target, move down
            else if (currentHeight > 0) {
                System.out.println("Moving down.");
                elevatorSubsystem.elevatorLeft.set(elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
                elevatorSubsystem.elevatorRight.set(-elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
            }
            // 5. If we’re within normal operating range but below target, move up
            else if (currentHeight < 0) {
                System.out.println("Moving up.");
                elevatorSubsystem.elevatorLeft.set(-elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
                elevatorSubsystem.elevatorRight.set(elevatorSubsystem.elevatorUpDownSpeed * heightLimiter);
            }
        }
        // Step 2 disables brake mode, waits, then re-enables brake mode
        // This brings the elevator all the way down
        else if (handoffStep == 2) {
            if (handoffStartTime < 0) {
                elevatorSubsystem.elevatorLeft.set(0);
                elevatorSubsystem.elevatorRight.set(0);
                elevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Coast);
                elevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Coast);

                handoffStartTime = Timer.getFPGATimestamp();
                return;
            }
        
            if (Timer.getFPGATimestamp() - handoffStartTime >= downBrakeDelay) {
                elevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                elevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);

                handoffStartTime = -1;
                handoffStep++;
            }
        }
        // Step 3 angles for intake
        else if (handoffStep == 3) {
            double target = -1.44;
            if (moveAngleToTarget(target)) {
                handoffStep++;
            }
        }
        // Step 4 passes from the funnel to coralator                
        else if (handoffStep == 4) {
            if (coralSubsystem.funnelSensor.getDistance(true).getValueAsDouble() > 0.1) {
                coralSubsystem.funnelLeft.set(0.0);
                coralSubsystem.funnelRight.set(0.0);
            }
            else {
                coralSubsystem.funnelLeft.set(-coralSubsystem.funnelSpeed);
                coralSubsystem.funnelRight.set(coralSubsystem.funnelSpeed);
            }

            if (coralSubsystem.coralForwardSensor.getDistance(true).getValueAsDouble() < 0.1) {
                coralSubsystem.flywheelMotor.set(0.0);
            }
            else {
                coralSubsystem.flywheelMotor.set(-coralSubsystem.flywheelOutSpeed);
            }

            if ((coralSubsystem.funnelSensor.getDistance(true).getValueAsDouble() > 0.1) && (coralSubsystem.coralForwardSensor.getDistance(true).getValueAsDouble() < 0.1)) {
                coralSubsystem.funnelLeft.set(0.0);
                coralSubsystem.funnelRight.set(0.0);
                coralSubsystem.flywheelMotor.set(0.0);
                handoffStep++;
            }
        }
        // Step 5 angles for movement and elevator
        else if (handoffStep == 5) {
            double target = -9;
            if (moveAngleToTarget(target)) {
                handoffStep++;
                isFinishedToggle = true;
            }
        }
        else {
            isFinishedToggle = true;
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

    private boolean moveAngleToTarget(double target) {
        double currentAngle = coralSubsystem.angleMotor.getPosition().getValueAsDouble();
        double tolerance = 0.05;
    
        if (Math.abs(currentAngle - target) <= tolerance) {
            System.out.println("At target. Stopping.");
            coralSubsystem.angleMotor.set(0.0);
            return true;
        }
    
        if (currentAngle > target) {
            if (currentAngle <= coralSubsystem.krakenAngleLowerLimit) {
                System.out.println("At/below lower limit. Cannot move down. Stopping.");
                coralSubsystem.angleMotor.set(0.0);
                return true;
            }
    
            System.out.println("Moving Down.");
            coralSubsystem.angleMotor.set(-coralSubsystem.angleSpeed);
        } else {
            if (currentAngle >= coralSubsystem.krakenAngleUpperLimit) {
                System.out.println("At/above upper limit. Cannot move up. Stopping.");
                coralSubsystem.angleMotor.set(0.0);
                return true;
            }
    
            System.out.println("Moving Up.");
            coralSubsystem.angleMotor.set(coralSubsystem.angleSpeed);
        }
    
        return false;
    }    
}
