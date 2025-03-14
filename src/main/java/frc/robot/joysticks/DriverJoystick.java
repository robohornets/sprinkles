package frc.robot.joysticks;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.coral.CoralController;
import frc.robot.subsystems.mechanisms.coral.CoralVariables;
import frc.robot.subsystems.mechanisms.elevator.ElevatorAutoHeight;
import frc.robot.subsystems.mechanisms.elevator.ElevatorController;
import frc.robot.subsystems.mechanisms.elevator.ElevatorVariables;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriverJoystick {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorController elevator;
    private final ElevatorVariables elevatorSubsystem;
    private final CoralController coral;
    private final CoralVariables coralSubsytem;
    
    public DriverJoystick(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
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
        // Reset field centric heading
        joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        
        // MARK: L Trigger
        joystick.leftTrigger()
            .whileTrue(RobotContainer.coral.flywheelIn())
            .onFalse(
                Commands.run(
                    () -> {
                        RobotContainer.coral.flywheelStop();
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
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );
        

        // MARK: D-Pad
        joystick.povUp()
            .whileTrue(coral.angleUp())
            .onFalse(
                Commands.run(
                    () -> {
                        CoralVariables.angleMotor.set(-0.015);
                        CoralVariables.angleMotor
                                .setNeutralMode(NeutralModeValue.Brake);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );
        

        joystick.povDown()
            .whileTrue(coral.angleDown())
            .onFalse(
                Commands.run(
                    () -> {
                        CoralVariables.angleMotor.set(-0.015);
                        CoralVariables.angleMotor
                                .setNeutralMode(NeutralModeValue.Brake);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        
        joystick.povLeft()
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

        joystick.povRight()
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