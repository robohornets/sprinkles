
package frc.robot.joysticks;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignOnTheFly;
import frc.robot.commands.Destinations;
import frc.robot.helpers.levelmanager.LevelManager;
import frc.robot.helpers.levelmanager.Levels;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.coral.CoralController;
import frc.robot.subsystems.mechanisms.coral.CoralVariables;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorController;
import frc.robot.subsystems.mechanisms.elevator.ElevatorVariables;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class MechBackup {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorController elevator;
    private final ElevatorVariables elevatorSubsystem;
    private final CoralController coral;
    private final CoralVariables coralSubsytem;
    
    public MechBackup(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
        ElevatorController elevator, ElevatorVariables elevatorSubsystem, CoralController coral, CoralVariables coralSubsystem) {

        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coral = coral;
        this.coralSubsytem = coralSubsystem;
    }

    public void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> RobotContainer.drive.withVelocityX(-joystick.getLeftY() * RobotContainer.MaxSpeed * 0.5)
                .withVelocityY(-joystick.getLeftX() * RobotContainer.MaxSpeed * 0.5)
                .withRotationalRate(-joystick.getRightX() * RobotContainer.MaxAngularRate)
            )
        );

        // MARK: Y-Button
        joystick.y()
            .whileTrue(coral.flywheelIn())
            .onFalse(
                Commands.run(
                    () -> {
                        CoralVariables.flywheelMotor.set(0.0);
                        CoralVariables.flywheelMotor.setNeutralMode(NeutralModeValue.Coast);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: X-Button
        joystick.x().onTrue(new AlignOnTheFly(Destinations.LEFT_REEF, drivetrain));

        // MARK: B-Button
        joystick.b().onTrue(new AlignOnTheFly(Destinations.RIGHT_REEF, drivetrain));

        // MARK: A-Button
        joystick.a().onTrue(new AlignOnTheFly(Destinations.COLLECTOR, drivetrain));
        
        // MARK: L Trigger
        joystick.leftTrigger()
            .whileTrue(coral.angleDown())
            .onFalse(
                Commands.run(
                    () -> {
                        CoralVariables.angleMotor.set(-0.015);
                        
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );
        
        // MARK: R Trigger
        joystick.rightTrigger()
            .whileTrue(coral.angleUp())
            .onFalse(
                Commands.run(
                    () -> {
                        CoralVariables.angleMotor.set(-0.015);
                        
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );
        
        // MARK: Left Bumper
        joystick.leftBumper()
            .whileTrue(elevator.elevatorDown())
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
        
        // MARK: D-Pad
        joystick.povDown().onTrue(new LevelManager(Levels.LEVEL_1, elevatorSubsystem, coralSubsytem).goToPreset());
        joystick.povLeft().onTrue(new LevelManager(Levels.LEVEL_2, elevatorSubsystem, coralSubsytem).goToPreset());
        joystick.povRight().onTrue(new LevelManager(Levels.LEVEL_3, elevatorSubsystem, coralSubsytem).goToPreset());
        joystick.povUp().onTrue(new LevelManager(Levels.LEVEL_4, elevatorSubsystem, coralSubsytem).goToPreset());
    }
}