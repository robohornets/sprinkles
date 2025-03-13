package frc.robot.subsystems.mechanisms.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanisms.coral.CoralVariables;
import frc.robot.subsystems.mechanisms.elevator.ElevatorVariables;

public class CoralSubsystem extends Command {
    private double angle;
    private CoralVariables coralSubsystem;
    public CoralSubsystem(double angle, CoralVariables coralSubsystem) {
        this.angle = angle;
        if (angle > 57) {
            this.angle = 57; 
        }
        else if (angle < 0) {
            this.angle = 0;
        }
        else {
            this.angle = angle;
        }
        this.coralSubsystem = coralSubsystem;
        addRequirements(this.coralSubsystem);
    }

    @Override
    public void initialize() {
        if (getCoralAngle() > 0.5) {
            System.out.println("Stopping");
            coralSubsystem.angleMotor.set(0.0);

            isFinishedToggle = true;
        }
        else if (getCoralAngle() < 0.5) {
            //ElevatorController.elevatorDown();
            System.out.println("Moving Down");
            coralSubsystem.angleMotor.set(ElevatorVariables.elevatorUpDownSpeed);
        }
        else if (coralSubsystem.angleMotor.getPosition().getValueAsDouble() > -angle) {
            System.out.println("Moving Up");
            coralSubsystem.angleMotor.set(CoralVariables.angleSpeed);
        }
    }

    @Override
    public void execute() {
        if (Math.abs(coralSubsystem.angleMotor.getPosition().getValueAsDouble() + angle) <= 2) {
            System.out.println("Stopping");
            coralSubsystem.angleMotor.set(0.0);
            coralSubsystem.angleMotor.set(0.0);

            isFinishedToggle = true;
            System.out.println(isFinishedToggle);
        }
    }

    private boolean isFinishedToggle = false;

    @Override
    public boolean isFinished() {
        System.out.println(isFinishedToggle);
        return isFinishedToggle;
    }

    public double getCoralAngle() {
        return CoralVariables.angleDCEncoder.get();
    }
}
