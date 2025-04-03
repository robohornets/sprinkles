package frc.robot.subsystems.mechanisms.coral.CommandManagers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;

public class CoralAngleManager extends Command {
    private double krakenAngle;
    private double pigeonAngle;
    private CoralSubsystem coralSubsystem;
    private boolean isFinishedToggle = false;
    private double oopsieThreshold = 0.01;
    private double krakenOopsieThreshold = 0.1;
    private double pigeonOopsieThreshold = 0.1;

    public CoralAngleManager(double krakenAngle, double pigeonAngle, CoralSubsystem coralSubsystem) {
        this.krakenAngle = krakenAngle;
        this.pigeonAngle = pigeonAngle;
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
        double krakenCurrentAngle = krakenGetCoralAngle();
        double pigeonCurrentAngle = pigeonGetCoralAngle();
    
        if (coralSubsystem.usePigeon ? (Math.abs(pigeonCurrentAngle-pigeonAngle) <= pigeonOopsieThreshold): (Math.abs(krakenCurrentAngle - krakenAngle) <= krakenOopsieThreshold)) {
            System.out.println("Stopping at target");
            isFinishedToggle = true;
            return;
        }
    
        if (coralSubsystem.usePigeon ? (pigeonCurrentAngle > coralSubsystem.pigeonAngleUpperLimit): (krakenCurrentAngle > coralSubsystem.krakenAngleUpperLimit)) {
            if (coralSubsystem.usePigeon ? (pigeonAngle < pigeonCurrentAngle): (krakenAngle < krakenCurrentAngle)) {
                System.out.println("Above upper limit, moving down");
                coralSubsystem.angleMotor.set(-coralSubsystem.angleSpeed);
            } else {
                System.out.println("Above upper limit, stopping");
                isFinishedToggle = true;
            }
        } else if (coralSubsystem.usePigeon ? (pigeonCurrentAngle < coralSubsystem.pigeonAngleLowerLimit): (krakenCurrentAngle < coralSubsystem.krakenAngleLowerLimit)) {
            if (coralSubsystem.usePigeon ? (pigeonAngle > pigeonCurrentAngle): krakenAngle > krakenCurrentAngle) {
                System.out.println("Below lower limit, moving up");
                coralSubsystem.angleMotor.set(coralSubsystem.angleSpeed);
            } else {
                System.out.println("Below lower limit, stopping");
                isFinishedToggle = true;
            }
        } else if (coralSubsystem.usePigeon ? (pigeonCurrentAngle > pigeonAngle): (krakenCurrentAngle > krakenAngle)) {
            System.out.println("Moving Down");
            coralSubsystem.angleMotor.set(-coralSubsystem.angleSpeed);
        } else if (coralSubsystem.usePigeon ? (pigeonCurrentAngle < pigeonAngle): (krakenCurrentAngle < krakenAngle)) {
            System.out.println("Moving Up");
            coralSubsystem.angleMotor.set(coralSubsystem.angleSpeed);
        }
    }    

    @Override
    public boolean isFinished() {
        return isFinishedToggle;
    }

    @Override
    public void end(boolean interrupted) {
        coralSubsystem.angleMotor.set(0.0);
        System.out.println("Command Ended. Motor Stopped.");
    }

    public double krakenGetCoralAngle() {
        return coralSubsystem.angleMotor.getPosition().getValueAsDouble();
    }
    public double pigeonGetCoralAngle() {
        return coralSubsystem.pigeonGetCoralAngle();
    }
}
