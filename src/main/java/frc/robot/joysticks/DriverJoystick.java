package frc.robot.joysticks;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.helpers.levelmanager.LevelManager;
import frc.robot.helpers.levelmanager.Levels;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.coral.CoralController;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorController;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriverJoystick {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorController elevator;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralController coral;
    private final CoralSubsystem coralSubsytem;
    
    public DriverJoystick(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
        ElevatorController elevator, ElevatorSubsystem elevatorSubsystem, CoralController coral, CoralSubsystem coralSubsystem) {

        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coral = coral;
        this.coralSubsytem = coralSubsystem;
    }

    public void configureBindings() {
        

        // MARK: Y-Button
        // Reset field centric heading
        joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        
        // MARK: L Trigger
        joystick.leftTrigger()
            .whileTrue(RobotContainer.coral.flywheelIn())
            .onFalse(
                Commands.run(
                    () -> {
                        RobotContainer.coral.flywheelStop();
                        CoralSubsystem.flywheelMotor.set(0.0);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );
        
        // MARK: R Trigger
        joystick.rightTrigger()
            .whileTrue(RobotContainer.coral.flywheelOut())
            .onFalse(
                Commands.run(
                    () -> {
                        RobotContainer.coral.flywheelStop();
                        CoralSubsystem.flywheelMotor.set(0.0);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );
        
        // MARK: Left Bumper
        joystick.leftBumper().onTrue(new LevelManager(Levels.CORAL_STATION, elevatorSubsystem, coralSubsytem).goToPreset());

        // MARK: Right Bumper
        joystick.rightBumper().onTrue(new LevelManager(Levels.DEFAULT_POSITION, elevatorSubsystem, coralSubsytem).goToPreset());
        

        // MARK: D-Pad
        joystick.povUp()
            .whileTrue(coral.angleUpSlow())
            .onFalse(
                Commands.run(
                    () -> {
                        CoralSubsystem.angleMotor.set(-0.015);
                        CoralSubsystem.angleMotor
                                .setNeutralMode(NeutralModeValue.Brake);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );
        

        joystick.povDown()
            .whileTrue(coral.angleDownSlow())
            .onFalse(
                Commands.run(
                    () -> {
                        CoralSubsystem.angleMotor.set(-0.015);
                        CoralSubsystem.angleMotor
                                .setNeutralMode(NeutralModeValue.Brake);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        
        joystick.povLeft()
            .whileTrue(elevator.elevatorDownSlow())
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

        joystick.povRight()
            .whileTrue(elevator.elevatorUpSlow())
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
    }
}