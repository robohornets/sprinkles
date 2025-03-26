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
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;

public class DebugJoystick {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralSubsystem coralSubsytem;
    private final AlgaeSubsystem algaeSubsystem;
    
    public DebugJoystick(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
        ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem, AlgaeSubsystem algaeSubsystem) {

        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsytem = coralSubsystem;
        this.algaeSubsystem = algaeSubsystem;
    }

    public void configureBindings() {
        // MARK: A-Button
        joystick.a()
            .whileTrue(coralSubsytem.flywheelOut())
            .onFalse(
                Commands.run(
                    () -> {
                        coralSubsytem.flywheelMotor.set(0.0);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        // MARK: B-Button
        
        joystick.b()
            .whileTrue(coralSubsytem.flywheelIn())
            .onFalse(
                Commands.run(
                    () -> {
                        coralSubsytem.flywheelMotor.set(0.0);
                        
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
        .whileTrue(algaeSubsystem.flywheelAlgaeIn())
        .onFalse(
            Commands.run(
                () -> {
                    algaeSubsystem.flywheelAlgaeMotor.set(0.0);
                    
                    CommandScheduler.getInstance().cancelAll();
                }
            )
        );

        // MARK: Right Trigger
        joystick.rightTrigger()
        .whileTrue(algaeSubsystem.flywheelAlgaeOut())
        .onFalse(
            Commands.run(
                () -> {
                    algaeSubsystem.flywheelAlgaeMotor.set(0.0);
                    
                    CommandScheduler.getInstance().cancelAll();
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

                        elevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                        elevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);

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

                        elevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
                        elevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);

                        CommandScheduler.getInstance().cancelAll();

                    }
                )
            );

        joystick.povDown().onTrue(new LevelManager(Levels.LEVEL_1, elevatorSubsystem, coralSubsytem).goToPreset());
        joystick.povLeft().onTrue(new LevelManager(Levels.LEVEL_2, elevatorSubsystem, coralSubsytem).goToPreset());
        joystick.povRight().onTrue(new LevelManager(Levels.LEVEL_3, elevatorSubsystem, coralSubsytem).goToPreset());
        joystick.povUp().onTrue(new LevelManager(Levels.LEVEL_4, elevatorSubsystem, coralSubsytem).goToPreset());
    }
}
