
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
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class MechBackup {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralSubsystem coralSubsystem;
    private final AlgaeSubsystem algaeSubsytem;
    
    public MechBackup(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
        ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem, AlgaeSubsystem algaeSubsystem) {

        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
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

        joystick.x().onTrue(new LevelManager(Levels.CORAL_STATION, elevatorSubsystem, coralSubsystem).goToPreset());


        // MARK: Coral Intake
        joystick.leftTrigger()
            .whileTrue(coralSubsystem.flywheelIn())
            .onFalse(
                Commands.run(
                    () -> {
                        coralSubsystem.flywheelMotor.set(0.0);
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        
        // MARK: Elevator U/D
        joystick.povDown()
            .whileTrue(elevatorSubsystem.elevatorDown())
            .onFalse(
                Commands.run(
                    () -> {
                        elevatorSubsystem.elevatorLeft.set(-0.015);
                        elevatorSubsystem.elevatorRight.set(0.015);
                        
                        CommandScheduler.getInstance().cancelAll();
                    }
                )
            );

        joystick.povUp()
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
        
        
        // joystick.povDown().onTrue(new LevelManager(Levels.LEVEL_1, elevatorSubsystem, coralSubsystem).goToPreset());
        // joystick.povLeft().onTrue(new LevelManager(Levels.LEVEL_2, elevatorSubsystem, coralSubsystem).goToPreset());
        // joystick.povRight().onTrue(new LevelManager(Levels.LEVEL_3, elevatorSubsystem, coralSubsystem).goToPreset());
        // joystick.povUp().onTrue(new LevelManager(Levels.LEVEL_4, elevatorSubsystem, coralSubsystem).goToPreset());

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