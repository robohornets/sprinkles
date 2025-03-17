
package frc.robot.joysticks;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignOnTheFlyClosest;
import frc.robot.commands.Destinations;
import frc.robot.helpers.levelmanager.LevelManager;
import frc.robot.helpers.levelmanager.Levels;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.coral.CoralController;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorController;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class MechBackup {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorController elevator;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralController coral;
    private final CoralSubsystem coralSubsytem;
    
    public MechBackup(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
        ElevatorController elevator, ElevatorSubsystem elevatorSubsystem, CoralController coral, CoralSubsystem coralSubsystem) {

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
                        CoralSubsystem.flywheelMotor.set(0.0);
                        CoralSubsystem.flywheelMotor.setNeutralMode(NeutralModeValue.Coast);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: X-Button
        joystick.x().onTrue(new AlignOnTheFlyClosest(Destinations.LEFT_REEF, drivetrain));

        // MARK: B-Button
        joystick.b().onTrue(new AlignOnTheFlyClosest(Destinations.RIGHT_REEF, drivetrain));

        // MARK: A-Button
        joystick.a().onTrue(new AlignOnTheFlyClosest(Destinations.COLLECTOR, drivetrain));
        
        // MARK: L Trigger
        joystick.leftTrigger()
            .whileTrue(coral.angleDown())
            .onFalse(
                Commands.run(
                    () -> {
                        CoralSubsystem.angleMotor.set(-0.015);
                        
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
                        CoralSubsystem.angleMotor.set(-0.015);
                        
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
                        ElevatorSubsystem.elevatorLeft.set(-0.015);
                        ElevatorSubsystem.elevatorRight.set(0.015);

                        ElevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                        ElevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                        
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
                        ElevatorSubsystem.elevatorLeft.set(-0.015);
                        ElevatorSubsystem.elevatorRight.set(0.015);

                        ElevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                        ElevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);

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