package frc.robot.joysticks.Debug;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignOnTheFlyByPose;
import frc.robot.commands.AlignOnTheFlyClosest;
import frc.robot.commands.Destinations;
import frc.robot.helpers.levelmanager.Levels;
import frc.robot.helpers.levelmanager.LevelManager;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;

public class DebugJoystick {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralSubsystem coralSubsystem;
    private final AlgaeSubsystem algaeSubsystem;
    
    public DebugJoystick(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
        ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem, AlgaeSubsystem algaeSubsystem) {

        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
        this.algaeSubsystem = algaeSubsystem;
    }

    public void configureBindings() {
        // MARK: A-Button
        joystick.a()
            .whileTrue(coralSubsystem.flywheelOut())
            .onFalse(
                Commands.run(
                    () -> {
                        coralSubsystem.flywheelMotor.set(0.0);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: B-Button
        
        joystick.b()
            .whileTrue(coralSubsystem.flywheelIn())
            .onFalse(
                Commands.run(
                    () -> {
                        coralSubsystem.flywheelMotor.set(0.0);
                        
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );
        

        // MARK: Y-Button
        // Aligns to the left side of the reef
    //    joystick.y().onTrue(new AlignOnTheFlyClosest(Destinations.RIGHT_REEF, drivetrain));


        // MARK: Start
        // joystick.start()
        // .onTrue(
        //     Commands.run(
        //         () -> {
        //             coralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Coast);
        //             coralSubsystem.flywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        //             coralSubsystem.funnelLeft.setNeutralMode(NeutralModeValue.Coast);
        //             coralSubsystem.funnelRight.setNeutralMode(NeutralModeValue.Coast);
        //             algaeSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Coast);
        //             elevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Coast);
        //             elevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Coast);
        //         }
        //     )
        // );

        joystick.back()
        .onTrue(
            Commands.run(
                () -> {
                    coralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
                    coralSubsystem.flywheelMotor.setNeutralMode(NeutralModeValue.Brake);
                    coralSubsystem.funnelLeft.setNeutralMode(NeutralModeValue.Brake);
                    coralSubsystem.funnelRight.setNeutralMode(NeutralModeValue.Brake);
                    algaeSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
                    elevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                    elevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
                }
            )
        );


        // MARK: Left Bumper
        joystick.leftBumper()
            .whileTrue(elevatorSubsystem.elevatorDown())
            .onFalse(Commands.run(
                    () -> {
                        elevatorSubsystem.elevatorLeft.set(-0.015);
                        elevatorSubsystem.elevatorRight.set(0.015);

                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: Right Bumper
        joystick.rightBumper()
            .whileTrue(elevatorSubsystem.elevatorUp())
            .onFalse(
                Commands.run(
                    () -> {
                        elevatorSubsystem.elevatorLeft.set(-0.015);
                        elevatorSubsystem.elevatorRight.set(0.015);

                        CommandScheduler.getInstance().cancelAll();

                    }
                )
            );

        
            joystick.leftTrigger()
            .whileTrue(coralSubsystem.angleDown())
            .onFalse(Commands.run(
                    () -> {
                        coralSubsystem.angleMotor.set(coralSubsystem.angleHoldSpeed);

                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: Right Bumper
        joystick.rightTrigger()
            .whileTrue(coralSubsystem.angleUp())
            .onFalse(
                Commands.run(
                    () -> {
                        coralSubsystem.angleMotor.set(coralSubsystem.angleHoldSpeed);

                        CommandScheduler.getInstance().cancelAll();

                    }
                )
            );

        joystick.povDown().onTrue(new LevelManager(Levels.LEVEL_1, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povLeft().onTrue(new LevelManager(Levels.LEVEL_2, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povRight().onTrue(new LevelManager(Levels.LEVEL_3, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povUp().onTrue(new LevelManager(Levels.LEVEL_4, elevatorSubsystem, coralSubsystem).goToPreset());
    }
}
