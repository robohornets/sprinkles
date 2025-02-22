package frc.robot.namedcommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.mechanisms.elevator.ElevatorVariables;

public class AutoNamedCommands {

    public static Command goToLevel1() {
        return new SequentialCommandGroup(
            new PrintCommand("Going to level 1"),

            // RunCommand that adjusts motor speed until we're in the 14.8–15.2 range
            new RunCommand(() -> {
                double currentPosition =
                    ElevatorVariables.elevatorLeft.getPosition().getValueAsDouble();

                // If below range, move elevator up
                if (currentPosition < 14.8) {
                    ElevatorVariables.elevatorLeft.set(-0.5);
                    ElevatorVariables.elevatorRight.set(0.5);
                }
                // If above range, move elevator down
                else if (currentPosition > 15.2) {
                    ElevatorVariables.elevatorLeft.set(0.5);
                    ElevatorVariables.elevatorRight.set(-0.5);
                }
                // If within 14.8–15.2, hold (stop motor)
                else {
                    ElevatorVariables.elevatorLeft.set(0.0);
                    ElevatorVariables.elevatorRight.set(0.0);
                }
            })
            // Keep running the above lambda UNTIL the position is within 14.8–15.2
            .until(() -> {
                double currentPosition =
                    ElevatorVariables.elevatorLeft.getPosition().getValueAsDouble();
                return (currentPosition >= 14.8 && currentPosition <= 15.2);
            })
            // Once finished, ensure motor is stopped
            .andThen(
                Commands.run(
                    () -> {
                        ElevatorVariables.elevatorLeft.set(0.0);
                        ElevatorVariables.elevatorRight.set(0.0);
                    }
                )
            ),
            //.andThen(() -> ElevatorVariables.elevatorLeft.set(0.0)),

            new PrintCommand("Reached level 1")
        );
    }
}
