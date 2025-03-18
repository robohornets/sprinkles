package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.helpers.levelmanager.LevelManager;
import frc.robot.helpers.levelmanager.Levels;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.coral.CoralController;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorController;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;

public class ButtonConsole {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorController elevator;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralController coral;
    private final CoralSubsystem coralSubsystem;
    
    public ButtonConsole(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
        ElevatorController elevator, ElevatorSubsystem elevatorSubsystem, CoralController coral, CoralSubsystem coralSubsystem) {

        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coral = coral;
        this.coralSubsystem = coralSubsystem;
    }

    public void configureBindings() {
        // MARK: Button A
        Trigger positionATrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.1 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.1
        );
        positionATrigger.onTrue(Commands.none());

        // MARK: Button B
        Trigger positionBTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.2 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.2
        );
        positionBTrigger.onTrue(Commands.none());

        // MARK: Button C
        Trigger positionCTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.3 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.3
        );
        positionCTrigger.onTrue(Commands.none());

        // MARK: Button D
        Trigger positionDTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.4 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.4
        );
        positionDTrigger.onTrue(Commands.none());

        // MARK: Button E
        Trigger positionETrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.5 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.5
        );
        positionETrigger.onTrue(Commands.none());

        // MARK: Button F
        Trigger positionFTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.6 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.6
        );
        positionFTrigger.onTrue(Commands.none());

        // MARK: Button G
        Trigger positionGTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.7 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.7
        );
        positionGTrigger.onTrue(Commands.none());

        // MARK: Button H
        Trigger positionHTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.8 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.8
        );
        positionHTrigger.onTrue(Commands.none());

        // MARK: Button I
        Trigger positionITrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.9 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.9
        );
        positionITrigger.onTrue(Commands.none());

        // MARK: Button J
        Trigger positionJTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == -0.1 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == -0.9
        );
        positionJTrigger.onTrue(Commands.none());

        // MARK: Button K
        Trigger positionKTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == -0.2 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == -0.8
        );
        positionKTrigger.onTrue(Commands.none());

        // MARK: Button L
        Trigger positionLTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == -0.3 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == -0.7
        );
        positionLTrigger.onTrue(Commands.none());

        // MARK: DPAD Bindings
        joystick.povDown().onTrue(new LevelManager(Levels.LEVEL_1, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povLeft().onTrue(new LevelManager(Levels.LEVEL_2, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povRight().onTrue(new LevelManager(Levels.LEVEL_3, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povUp().onTrue(new LevelManager(Levels.LEVEL_4, elevatorSubsystem, coralSubsystem).goToPreset());
    }    
}
