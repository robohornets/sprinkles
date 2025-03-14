package frc.robot.joysticks;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignOnTheFly;
import frc.robot.commands.Destinations;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.coral.CoralController;
import frc.robot.subsystems.mechanisms.coral.CoralVariables;
import frc.robot.subsystems.mechanisms.elevator.ElevatorController;
import frc.robot.subsystems.mechanisms.elevator.ElevatorVariables;

public class DebugJoystick {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorController elevator;
    private final ElevatorVariables elevatorSubsystem;
    private final CoralController coral;
    private final CoralVariables coralSubsytem;
    
    public DebugJoystick(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
        ElevatorController elevator, ElevatorVariables elevatorSubsystem, CoralController coral, CoralVariables coralSubsystem) {

        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coral = coral;
        this.coralSubsytem = coralSubsystem;
    }

    public void configureBindings() {
        // MARK: A-Button
        joystick.a().onTrue(new AlignOnTheFly(Destinations.LEFT_REEF, drivetrain));

        // MARK: B-Button
        joystick.b().onTrue(new AlignOnTheFly(Destinations.RIGHT_REEF, drivetrain));

        // MARK: X-Button
        joystick.x().onTrue(new AlignOnTheFly(Destinations.COLLECTOR, drivetrain));

        // MARK: Y-Button


        // MARK: Left Trigger
        joystick.leftTrigger()
            .whileTrue(coral.angleDown())
            .onFalse(
                Commands.run(
                    () -> {
                        CoralVariables.angleMotor.set(-0.015);
                        CoralVariables.angleMotor.setNeutralMode(NeutralModeValue.Brake);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: Right Trigger
        joystick.rightTrigger()
            .whileTrue(coral.angleUp())
            .onFalse(
                Commands.run(
                    () -> {
                        CoralVariables.angleMotor.set(-0.015);
                        CoralVariables.angleMotor.setNeutralMode(NeutralModeValue.Brake);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: Left Bumper
        joystick.leftBumper()
            .whileTrue(elevator.elevatorDown())
            .onFalse(Commands.run(
                    () -> {
                        ElevatorVariables.elevatorLeft.set(-0.015);
                        ElevatorVariables.elevatorRight.set(0.015);

                        ElevatorVariables.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                        ElevatorVariables.elevatorRight.setNeutralMode(NeutralModeValue.Brake);

                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: Right Bumper
        joystick.rightBumper()
            .whileTrue(elevator.elevatorUp())
            .onFalse(
                Commands.run(
                    () -> {
                        ElevatorVariables.elevatorLeft.set(-0.015);
                        ElevatorVariables.elevatorRight.set(0.015);

                        ElevatorVariables.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                        ElevatorVariables.elevatorRight.setNeutralMode(NeutralModeValue.Brake);

                        CommandScheduler.getInstance().cancelAll();

                    }
                )
            );
    }
}
