package frc.robot.joysticks;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.helpers.levelmanager.LevelManager;
import frc.robot.helpers.levelmanager.Levels;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;


public class DriverJoystick {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralSubsystem coralSubsystem;
    private final AlgaeSubsystem algaeSubsytem;
    
    public DriverJoystick(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
        ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem, AlgaeSubsystem algaeSubsystem) {

        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
        this.algaeSubsytem = algaeSubsystem;
    }

    public void configureBindings() {
        

        // MARK: Y-Button
        // Reset field centric heading
        joystick.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // joystick.x().onTrue(new InstantCommand(() -> RobotContainer.setUseFieldCentric(false)));
       
        joystick.y()
        .whileTrue(algaeSubsytem.angleAlgaeUp())
        .onFalse(
            Commands.run(
                () -> {
                    algaeSubsytem.angleAlgaeMotor.set(0.0);
                }
            )
        );

    joystick.a()
        .whileTrue(algaeSubsytem.angleAlgaeDown())
        .onFalse(
            Commands.run(
                () -> {
                    algaeSubsytem.angleAlgaeMotor.set(0.0);
                }
            )
        );

        joystick.b().onTrue(new LevelManager(Levels.CORAL_STATION, elevatorSubsystem, coralSubsystem).goToPreset());
        
        // MARK: L Trigger
        joystick.leftTrigger()
        .whileTrue(algaeSubsytem.flywheelAlgaeOut())
        .onFalse(
            Commands.run(
                () -> {
                    algaeSubsytem.flywheelAlgaeMotor.set(0.0);
                    
                    CommandScheduler.getInstance().cancelAll();
                }
            )
        );
        
        // MARK: R Trigger
        joystick.rightTrigger()
            .whileTrue(coralSubsystem.flywheelOut())
            .onFalse(
                Commands.run(
                    () -> {
                        coralSubsystem.flywheelStop();
                        coralSubsystem.flywheelMotor.set(0.0);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: Left Bumper
        joystick.leftBumper()
            .whileTrue(coralSubsystem.angleDownSlow())
            .onFalse(
                Commands.run(
                    () -> {
                        coralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );
        
        // MARK: Right Bumper
        joystick.rightBumper()
            .whileTrue(coralSubsystem.angleUpSlow())
            .onFalse(
                Commands.run(
                    () -> {
                        coralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
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

        joystick.povUp()
            .whileTrue(drivetrain.applyRequest(
                () -> RobotContainer.driveRobotCentric
                    .withVelocityX(RobotContainer.MaxSpeed * 0.15)
                    .withVelocityY(0)
            ));
    }
}