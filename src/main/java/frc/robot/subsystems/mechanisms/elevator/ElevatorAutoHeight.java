package frc.robot.subsystems.mechanisms.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ElevatorAutoHeight extends Command {
    private double height;
    private ElevatorVariables elevatorSubsystem;
    public ElevatorAutoHeight(double height, ElevatorVariables elevatorSubsystem) {
        this.height = height;
        if (height > 57) {
            this.height = 57; 
        }
        else if (height < 0) {
            this.height = 0;
        }
        else {
            this.height = height;
        }
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void initialize() {
        if (Math.abs(elevatorSubsystem.elevatorLeft.getPosition().getValueAsDouble() + height) <= 2) {
            System.out.println("Stopping");
            elevatorSubsystem.elevatorLeft.set(0.0);
            elevatorSubsystem.elevatorRight.set(0.0);

            isFinishedToggle = true;
        }
        else if (elevatorSubsystem.elevatorLeft.getPosition().getValueAsDouble() < -height) {
            //ElevatorController.elevatorDown();
            System.out.println("Moving Down");
            elevatorSubsystem.elevatorLeft.set(ElevatorVariables.elevatorUpDownSpeed);
            elevatorSubsystem.elevatorRight.set(-ElevatorVariables.elevatorUpDownSpeed);
        }
        else if (elevatorSubsystem.elevatorLeft.getPosition().getValueAsDouble() > -height) {
            System.out.println("Moving Up");
            elevatorSubsystem.elevatorLeft.set(-ElevatorVariables.elevatorUpDownSpeed);
            elevatorSubsystem.elevatorRight.set(ElevatorVariables.elevatorUpDownSpeed);
        }
        // else {
        //     elevatorSubsystem.elevatorLeft.set(0.0);
        //     elevatorSubsystem.elevatorRight.set(0.0);

        //     isFinishedToggle = true;
        // }
    }

    @Override
    public void execute() {
        if (Math.abs(elevatorSubsystem.elevatorLeft.getPosition().getValueAsDouble() + height) <= 2) {
            System.out.println("Stopping");
            elevatorSubsystem.elevatorLeft.set(0.0);
            elevatorSubsystem.elevatorRight.set(0.0);

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
}
