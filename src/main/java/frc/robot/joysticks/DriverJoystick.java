package frc.robot.joysticks;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.helpers.levelmanager.LevelManager;
import frc.robot.helpers.levelmanager.Levels;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.coral.CommandManagers.HandoffManager;
import frc.robot.subsystems.mechanisms.coral.CommandManagers.InOutCommands.CoralOutCommand;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class DriverJoystick {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralSubsystem coralSubsystem;
    private final AlgaeSubsystem algaeSubsystem;
    
    public DriverJoystick(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
        ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem, AlgaeSubsystem algaeSubsystem) {

        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
        this.algaeSubsystem = algaeSubsystem;
    }

    public void configureBindings() {
        
       
        // MARK: B - Handoff
        joystick.b()
            .onTrue(
                new HandoffManager(coralSubsystem, elevatorSubsystem)
            );

        // MARK: X - Algae Up
        joystick.x()
            .whileTrue(algaeSubsystem.angleAlgaeUp())
            .onFalse(
                Commands.run(
                    () -> {
                        algaeSubsystem.angleMotor.set(0.0);
                    }
                )
            );

        // MARK: Y - Algae Down
        joystick.x()
            .whileTrue(algaeSubsystem.angleAlgaeDown())
            .onFalse(
                Commands.run(
                    () -> {
                        algaeSubsystem.angleMotor.set(0.0);
                    }
                )
            );
        
        
        // MARK: LT - Algae Out
        joystick.rightTrigger()
            .whileTrue(
                Commands.run(
                    () -> {
                        
                    }
                )
            )
            .onFalse(
                Commands.run(
                    () -> {
                        
                    }
                )
            );
        
        // MARK: RT - Coral Out
        joystick.rightTrigger()
            .onTrue(new CoralOutCommand(coralSubsystem));

        // MARK: LB - Coral Down
        joystick.leftBumper()
            .whileTrue(coralSubsystem.angleDownSlow())
            .onFalse(
                Commands.run(
                    () -> {
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );
        
        // MARK: RB - Coral Up
        joystick.rightBumper()
            .whileTrue(coralSubsystem.angleUpSlow())
            .onFalse(
                Commands.run(
                    () -> {
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: L - RoboCentric
        joystick.povLeft()
            .whileTrue(drivetrain.applyRequest(
                () -> RobotContainer.driveRobotCentric
                    .withVelocityX(0)
                    .withVelocityY(RobotContainer.MaxSpeed * 0.15)
                )
            );

        // MARK: R - RoboCentric
        joystick.povRight()
            .whileTrue(
                drivetrain.applyRequest(
                    () -> RobotContainer.driveRobotCentric
                        .withVelocityX(0)
                        .withVelocityY(-RobotContainer.MaxSpeed * 0.15)
                )
            );

        // MARK: U - RoboCentric
        joystick.povUp()
            .whileTrue(drivetrain.applyRequest(
                () -> RobotContainer.driveRobotCentric
                    .withVelocityX(RobotContainer.MaxSpeed * 0.15)
                    .withVelocityY(0)
                )
            );
        
        // MARK: D - Reset FC
        joystick.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }
}