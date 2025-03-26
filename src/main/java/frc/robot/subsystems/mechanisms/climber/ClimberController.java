package frc.robot.subsystems.mechanisms.climber;

public class ClimberController {
    public void elevatorUp() {
        ClimberVariables.alexHonnold.set(0.5);
    }

    public void elevatorDown() {
        ClimberVariables.alexHonnold.set(-0.5);
    }

    public void stopElevator() {
        ClimberVariables.alexHonnold.set(0.0);
    }
}
