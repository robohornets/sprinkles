
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
import frc.robot.subsystems.mechanisms.algae.AlgaeAngleManager;
import frc.robot.subsystems.mechanisms.algae.AlgaeController;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;
import frc.robot.subsystems.mechanisms.climber.ClimberController;
import frc.robot.subsystems.mechanisms.climber.ClimberVariables;
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
    private final AlgaeController algae;
    private final AlgaeSubsystem algaeSubsytem;
    
    public MechBackup(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
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
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> RobotContainer.drive.withVelocityX(-joystick.getLeftY() * RobotContainer.MaxSpeed * 0.5)
                .withVelocityY(-joystick.getLeftX() * RobotContainer.MaxSpeed * 0.5)
                .withRotationalRate(-joystick.getRightX() * RobotContainer.MaxAngularRate)
            )
        );

        // MARK: AutoAlign
        joystick.leftBumper().onTrue(new AlignOnTheFlyClosest(Destinations.LEFT_REEF, drivetrain));
        joystick.rightBumper().onTrue(new AlignOnTheFlyClosest(Destinations.RIGHT_REEF, drivetrain));        
        joystick.a().onTrue(new AlignOnTheFlyClosest(Destinations.COLLECTOR, drivetrain));


        // MARK: Coral Intake
        joystick.rightTrigger()
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

        // MARK: Algae Intake
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
        
        // MARK: Elevator U/D
        joystick.povDown()
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

        joystick.povUp()
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
        
        
        // joystick.povDown().onTrue(new LevelManager(Levels.LEVEL_1, elevatorSubsystem, coralSubsytem).goToPreset());
        // joystick.povLeft().onTrue(new LevelManager(Levels.LEVEL_2, elevatorSubsystem, coralSubsytem).goToPreset());
        // joystick.povRight().onTrue(new LevelManager(Levels.LEVEL_3, elevatorSubsystem, coralSubsytem).goToPreset());
        // joystick.povUp().onTrue(new LevelManager(Levels.LEVEL_4, elevatorSubsystem, coralSubsytem).goToPreset());

                // 
                // joystick.povDown()
                // .whileTrue(coral.angleDown())
                // .onFalse(
                //     Commands.run(
                //         () -> {
                //             CoralSubsystem.angleMotor.set(-0.015);
                            
                //             CommandScheduler.getInstance().cancelAll();
                //         }
                //     )
                // );
            
            // joystick.povUp()
            //     .whileTrue(coral.angleUp())
            //     .onFalse(
            //         Commands.run(
            //             () -> {
            //                 CoralSubsystem.angleMotor.set(-0.015);
                            
            //                 CommandScheduler.getInstance().cancelAll();
            //             }
            //         )
            //     );


            // joystick.povLeft()
            // .whileTrue(algae.angleAlgaeDown())
            // .onFalse(
            //     Commands.run(
            //         () -> {
            //             algaeSubsytem.angleAlgaeMotor.set(0.0);
                        
            //             CommandScheduler.getInstance().cancelAll();
            //         }
            //     )
            // );

            // joystick.leftTrigger()
            //     .onTrue(
            //         new AlignOnTheFlyClosest(Destinations.LEFT_REEF, drivetrain)
            //     );

            // joystick.a().whileTrue(
            //     Commands.run(
            //         () -> {
            //             ClimberVariables.alexHonnold.set(0.5);
            //         }
            //     )
            // );
    
            // joystick.b().whileTrue(
            //     Commands.run(
            //         () -> {
            //             ClimberVariables.alexHonnold.set(-0.5);
            //         }
            //     )
            // );
            

            // joystick.povRight()
            //     .whileTrue(algae.angleAlgaeUp())
            //     .onFalse(
            //         Commands.run(
            //             () -> {
            //                 algaeSubsytem.angleAlgaeMotor.set(0.0);
                            
            //                 CommandScheduler.getInstance().cancelAll();
            //             }
            //         )
            //     );
    }
}