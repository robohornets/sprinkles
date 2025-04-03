package frc.robot.subsystems.mechanisms.coral;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;

public class CoralSubsystem extends SubsystemBase {

    //in and out flywheels 11, coral angle 12, algae 13, funnel wheels 14, remove seperate algae wheels and 
    //change coral wheels to in and out wheels (one motor controls both), might need to change id on angles

    public double angleUpperLimit = 0.79;
    public double angleLowerLimit = 0.32;

    public double krakenAngleUpperLimit = 0.0;
    public double krakenAngleLowerLimit = -9.5;

    public double pigeonAngleUpperLimit = 0.0;
    public double pigeonAngleLowerLimit = -9.5;

    public TalonFX angleMotor = new TalonFX(12);
    public TalonFX flywheelMotor = new TalonFX(11);

    public TalonFX funnelLeft = new TalonFX(14);
    public TalonFX funnelRight = new TalonFX(15);
    public DutyCycleEncoder angleDCEncoder = new DutyCycleEncoder(4);
    
    public double angleSpeed = 0.1;
    public double flywheelInSpeed = 0.2;
    public double flywheelOutSpeed = 0.4;
    public double funnelSpeed = 0.08;
    
    public Double angleHoldSpeed = 0.02;

    public Pigeon2 coralPigeon = new Pigeon2(37);
    public boolean usePigeon = false;

    public CANrange coralForwardSensor = new CANrange(35);
    public Trigger coralForwardTrigger = new Trigger(() -> 
        coralForwardSensor.getDistance(true).getValueAsDouble() < 0.1
    );

    public CANrange funnelSensor = new CANrange(36);
    public Trigger funnelTrigger = new Trigger(() -> 
        funnelSensor.getDistance(true).getValueAsDouble() < 0.2
    );

    // MARK: Flywheel Commands
    public Command flywheelOut() {
        return Commands.run(
            () -> {
                flywheelMotor.set(flywheelOutSpeed);
            }
        );
    }
    public Command flywheelIn() {
        return Commands.run(
            () -> {
                flywheelMotor.set(-flywheelInSpeed);
            }
        );
    }
    public Command flywheelStop() {
        return Commands.run(
            () -> {
                flywheelMotor.set(0.0);
            }
        );
    }

    // MARK: Angle Commands
    public Command angleUp() {
        return Commands.run(
            () -> {
                if (getLimitAsBool(true)) {
                    angleMotor.set(angleSpeed); }
                else {
                    angleMotor.set(angleHoldSpeed);
                }
            }
        );
    }

    public Command angleDown() {
        return Commands.run(
            () -> {
                if (getLimitAsBool(false)) {
                    angleMotor.set(-angleSpeed);} 
                else {
                    angleMotor.set(angleHoldSpeed);
                }
            }
        );
    }

    public Command angleUpSlow() {
        return Commands.run(
            () -> {
                if (getLimitAsBool(true)) {
                    angleMotor.set(-0.1); }
                else {
                    angleMotor.set(angleHoldSpeed);
                }
            }
        );
    }

    public Command angleDownSlow() {
        return Commands.run(
            () -> {
                if (getLimitAsBool(false)) {
                    angleMotor.set(0.1);} 
                else {
                    angleMotor.set(angleHoldSpeed);
                }
            }
        );
    }

    // MARK: Funnel Commands
    public Command funnelThrough(ElevatorSubsystem elevatorSubsystem) {
        return Commands.run(
            () -> {
                if (Math.abs(elevatorSubsystem.getElevatorHeight()) < 5) {
                    funnelLeft.set(funnelSpeed);
                    funnelRight.set(-funnelSpeed);
                } 
                else {
                    funnelLeft.set(0.0);
                    funnelRight.set(0.0);
                }
            }
        );
    }

    public Command funnelUneven(boolean isLeftSide) {
        return Commands.run(
            () -> {
                if (isLeftSide) {
                    funnelLeft.set(-0.1);
                    funnelRight.set(0.05);
                }
                else {
                    funnelLeft.set(-0.05);
                    funnelRight.set(0.1);
                }
            }
        );
    }


    public double getCoralAngle() {
        return angleDCEncoder.get();
    }

    public boolean coralAngleIsConnected() {
        return angleDCEncoder.isConnected();
    }

    public double krakenGetCoralAngle() {
        return angleMotor.getPosition().getValueAsDouble();
    }

    public boolean getLimitAsBool(boolean isUpperLimit) {
        if (isUpperLimit) {
            return krakenGetCoralAngle() < krakenAngleUpperLimit;
        }
        else {
            return krakenGetCoralAngle() > krakenAngleLowerLimit;
        }
    }

    public double pigeonGetCoralAngle() {
        return -coralPigeon.getRoll().getValueAsDouble();
    }
}
