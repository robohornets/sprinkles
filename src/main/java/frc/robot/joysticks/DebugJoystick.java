package frc.robot.joysticks;

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
import frc.robot.subsystems.mechanisms.coral.CoralController;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorController;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import frc.robot.subsystems.mechanisms.algae.AlgaeController;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;

public class DebugJoystick {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorController elevator;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralController coral;
    private final CoralSubsystem coralSubsytem;
    private final AlgaeController algae;
    private final AlgaeSubsystem algaeSubsystem;
    
    public DebugJoystick(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
        ElevatorController elevator, ElevatorSubsystem elevatorSubsystem, CoralController coral, CoralSubsystem coralSubsystem, AlgaeController algae, AlgaeSubsystem algaeSubsystem) {

        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coral = coral;
        this.coralSubsytem = coralSubsystem;
        this.algae = algae;
        this.algaeSubsystem = algaeSubsystem;
    }

    public void configureBindings() {
        // MARK: A-Button
        joystick.a()
            .whileTrue(coral.flywheelOut())
            .onFalse(
                Commands.run(
                    () -> {
                        CoralSubsystem.flywheelMotor.set(0.0);
                        CoralSubsystem.flywheelMotor.setNeutralMode(NeutralModeValue.Coast);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: B-Button
        joystick.b()
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
        

        // collect top algae by passing in Pose2d location
        joystick.y().onTrue(new AlignOnTheFlyByPose(new Pose2d(1.05, 6.4, new Rotation2d(216.0)), drivetrain));

        // MARK: X-Button
        // Aligns to the right side of the reef
        joystick.x().onTrue(new AlignOnTheFlyClosest(Destinations.LEFT_REEF, drivetrain));

        // MARK: Y-Button
        // Aligns to the left side of the reef
    //    joystick.y().onTrue(new AlignOnTheFlyClosest(Destinations.RIGHT_REEF, drivetrain));


        // MARK: Start
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // MARK: Back
        joystick.back().onTrue(new AlignOnTheFlyClosest(Destinations.COLLECTOR, drivetrain));

        // MARK: Left Trigger
        joystick.leftTrigger()
        .whileTrue(algae.flywheelAlgaeIn())
        .onFalse(
            Commands.run(
                () -> {
                    AlgaeSubsystem.flywheelAlgaeMotor.set(0.0);
                    AlgaeSubsystem.flywheelAlgaeMotor.setNeutralMode(NeutralModeValue.Coast);
                    CommandScheduler.getInstance().cancelAll();
                }
            )
        );

        // MARK: Right Trigger
        joystick.rightTrigger()
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

        // // MARK: Left Trigger
        // joystick.leftTrigger()
        //     .whileTrue(coral.angleDown())
        //     .onFalse(
        //         Commands.run(
        //             () -> {
        //                 CoralSubsystem.angleMotor.set(-0.015);
        //                 CoralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
        //                 CommandScheduler.getInstance().cancelAll();
        //             }
        //         )
        //     );

        // // MARK: Right Trigger
        // joystick.rightTrigger()
        //     .whileTrue(coral.angleUp())
        //     .onFalse(
        //         Commands.run(
        //             () -> {
        //                 CoralSubsystem.angleMotor.set(-0.015);
        //                 CoralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
        //                 CommandScheduler.getInstance().cancelAll();
        //             }
        //         )
        //     );

        // MARK: Left Bumper
        joystick.leftBumper()
            .whileTrue(elevator.elevatorDown())
            .onFalse(Commands.run(
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
    // joystick.povDown().onTrue(new ElevatorSubsystem(0.0, elevatorSubsystem));
    // joystick.povLeft().onTrue(new ElevatorSubsystem(17.0, elevatorSubsystem));
    // joystick.povRight().onTrue(new ElevatorSubsystem(37.0, elevatorSubsystem));
    // joystick.povUp().onTrue(new ElevatorSubsystem(65.0, elevatorSubsystem));
        // joystick.povDown().onTrue(new LevelManager(Levels.LEVEL_1, elevatorSubsystem, coralSubsytem).goToPreset());
        // joystick.povLeft().onTrue(new LevelManager(Levels.LEVEL_2, elevatorSubsystem, coralSubsytem).goToPreset());
        // joystick.povRight().onTrue(new LevelManager(Levels.LEVEL_3, elevatorSubsystem, coralSubsytem).goToPreset());
        // joystick.povUp().onTrue(new LevelManager(Levels.LEVEL_4, elevatorSubsystem, coralSubsytem).goToPreset());
        joystick.povDown()
        .whileTrue(algae.angleAlgaeDown())
        .onFalse(
            Commands.run(
                () -> {
                    AlgaeSubsystem.angleAlgaeMotor.set(-0.015);
                    AlgaeSubsystem.angleAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
                    CommandScheduler.getInstance().cancelAll();
                }
            )
        );
        joystick.povUp()
        .whileTrue(algae.angleAlgaeUp())
        .onFalse(
            Commands.run(
                () -> {
                    AlgaeSubsystem.angleAlgaeMotor.set(-0.015);
                    AlgaeSubsystem.angleAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
                    CommandScheduler.getInstance().cancelAll();
                }
            )
        );
    }
}
