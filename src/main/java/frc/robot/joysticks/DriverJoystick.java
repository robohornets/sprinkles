package frc.robot.joysticks;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.helpers.levelmanager.LevelManager;
import frc.robot.helpers.levelmanager.Levels;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.algae.AlgaeController;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;
import frc.robot.subsystems.mechanisms.coral.CoralController;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorController;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;


public class DriverJoystick {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorController elevator;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralController coral;
    private final CoralSubsystem coralSubsytem;
    private final AlgaeController algae;
    private final AlgaeSubsystem algaeSubsytem;
    
    public DriverJoystick(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
        ElevatorController elevator, ElevatorSubsystem elevatorSubsystem, CoralController coral, CoralSubsystem coralSubsystem, AlgaeController algae, AlgaeSubsystem algaeSubsystem) {

        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coral = coral;
        this.coralSubsytem = coralSubsystem;
        this.algae = algae;
        this.algaeSubsytem = algaeSubsystem;
    }

    public void configureBindings() {
        

        // MARK: Y-Button
        // Reset field centric heading
        joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // joystick.x().onTrue(new InstantCommand(() -> RobotContainer.setUseFieldCentric(false)));
       
        
        // MARK: L Trigger
        joystick.leftTrigger()
        .whileTrue(algae.flywheelAlgaeOut())
        .onFalse(
            Commands.run(
                () -> {
                    AlgaeSubsystem.flywheelAlgaeMotor.set(0.0);
                    AlgaeSubsystem.flywheelAlgaeMotor.setNeutralMode(NeutralModeValue.Coast);
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
        joystick.leftBumper()
            .whileTrue(coral.angleDownSlow())
            .onFalse(
                Commands.run(
                    () -> {
                        // CoralSubsystem.angleMotor.set(-0.015);
                        CoralSubsystem.angleMotor
                                .setNeutralMode(NeutralModeValue.Brake);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );
        
        // MARK: Right Bumper
        joystick.rightBumper()
            .whileTrue(coral.angleUpSlow())
            .onFalse(
                Commands.run(
                    () -> {
                        // CoralSubsystem.angleMotor.set(-0.015);
                        CoralSubsystem.angleMotor
                                .setNeutralMode(NeutralModeValue.Brake);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: D-Pad
        joystick.povLeft()
            .whileTrue(drivetrain.applyRequest(
                () -> RobotContainer.driveRobotCentric
                    .withVelocityX(0)
                    .withVelocityY(RobotContainer.MaxSpeed * 0.15)
            ));

        joystick.povRight()
            .whileTrue(drivetrain.applyRequest(
                () -> RobotContainer.driveRobotCentric
                    .withVelocityX(0)
                    .withVelocityY(-RobotContainer.MaxSpeed * 0.15)
            ));

        joystick.povDown()
            .whileTrue(drivetrain.applyRequest(
                () -> RobotContainer.driveRobotCentric
                    .withVelocityX(-RobotContainer.MaxSpeed * 0.15)
                    .withVelocityY(0)
            ));

        joystick.povUp()
            .whileTrue(drivetrain.applyRequest(
                () -> RobotContainer.driveRobotCentric
                    .withVelocityX(RobotContainer.MaxSpeed * 0.15)
                    .withVelocityY(0)
            ));
    }
}