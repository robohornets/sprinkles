package frc.robot.joysticks;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignOnTheFlyByPose;
import frc.robot.helpers.levelmanager.LevelManager;
import frc.robot.helpers.levelmanager.Levels;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.coral.CoralController;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorController;
import frc.robot.subsystems.mechanisms.elevator.ElevatorHeightManager;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;

public class ButtonConsole {
    private final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final ElevatorController elevator;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralController coral;
    private final CoralSubsystem coralSubsystem;
    Optional<Alliance> ally = DriverStation.getAlliance();
    
    public ButtonConsole(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, 
        ElevatorController elevator, ElevatorSubsystem elevatorSubsystem, CoralController coral, CoralSubsystem coralSubsystem) {

        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coral = coral;
        this.coralSubsystem = coralSubsystem;
    }

    public void configureBindings() {
        // MARK: Button A
        Trigger positionATrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.1 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.1
        );
        if(ally.isPresent()){
            if(ally.get() == Alliance.Blue){
                positionATrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(3.202, 4.004, new Rotation2d(Units.degreesToRadians(360))), drivetrain));
            }
            if(ally.get() == Alliance.Red){
                positionATrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(11.799, 4.025, new Rotation2d(Units.degreesToRadians(0))), drivetrain));
            }
        }
        else{
            positionATrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(3.202, 4.004, new Rotation2d(Units.degreesToRadians(360))), drivetrain));
        }

        // MARK: Button B
        Trigger positionBTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.2 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.2
        );
        if(ally.isPresent()){
            if(ally.get() == Alliance.Blue){
                positionBTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(3.201, 3.661, new Rotation2d(Units.degreesToRadians(360))), drivetrain));
            }
            if(ally.get() == Alliance.Red){
                positionBTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(11.799, 3.695, new Rotation2d(Units.degreesToRadians(0))), drivetrain));
            }
        }
        else{
            positionBTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(3.201, 3.661, new Rotation2d(Units.degreesToRadians(360))), drivetrain));
        }

        // MARK: Button C
        Trigger positionCTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.3 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.3
        );
        if(ally.isPresent()){
            if(ally.get() == Alliance.Blue){
                positionCTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(1.635, 3.392, new Rotation2d(Units.degreesToRadians(60))), drivetrain));
            }
            if(ally.get() == Alliance.Red){
                positionCTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(12.428, 2.931, new Rotation2d(Units.degreesToRadians(60))), drivetrain));
            }
        }
        else{
            positionCTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(1.635, 3.392, new Rotation2d(Units.degreesToRadians(60))), drivetrain));
        }


        // MARK: Button D
        Trigger positionDTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.4 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.4
        );
        if(ally.isPresent()){
            if(ally.get() == Alliance.Blue){
                positionDTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(4.138, 2.714, new Rotation2d(Units.degreesToRadians(60))), drivetrain));
            }
            if(ally.get() == Alliance.Red){
                positionDTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(12.728, 2.725, new Rotation2d(Units.degreesToRadians(60))), drivetrain));
            }
        }
        else{
            positionDTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(4.138, 2.714, new Rotation2d(Units.degreesToRadians(60))), drivetrain));
        }


        // MARK: Button E
        Trigger positionETrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.5 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.5
        );
        if(ally.isPresent()){
            if(ally.get() == Alliance.Blue){
                positionETrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(5.155, 2.883, new Rotation2d(Units.degreesToRadians(120))), drivetrain));
            }
            if(ally.get() == Alliance.Red){
                positionETrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(13.708, 2.910, new Rotation2d(Units.degreesToRadians(120))), drivetrain));
            }
        }
        else{
            positionETrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(5.155, 2.883, new Rotation2d(Units.degreesToRadians(120))), drivetrain));
        }


        // MARK: Button F
        Trigger positionFTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.6 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.6
        );
        if(ally.isPresent()){
            if(ally.get() == Alliance.Blue){
                positionFTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(5.474, 3.083, new Rotation2d(Units.degreesToRadians(120))), drivetrain));
            }
            if(ally.get() == Alliance.Red){
                positionFTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(13.997, 3.086, new Rotation2d(Units.degreesToRadians(120))), drivetrain));
            }
        }
        else{
            positionFTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(5.474, 3.083, new Rotation2d(Units.degreesToRadians(120))), drivetrain));
        }


        // MARK: Button G
        Trigger positionGTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.7 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.7
        );
        if(ally.isPresent()){
            if(ally.get() == Alliance.Blue){
                positionGTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(5.803, 4.060, new Rotation2d(Units.degreesToRadians(180))), drivetrain));
            }
            if(ally.get() == Alliance.Red){
                positionGTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(14.348, 4.035, new Rotation2d(Units.degreesToRadians(180))), drivetrain));
            }
        }
        else{
            positionGTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(5.803, 4.060, new Rotation2d(Units.degreesToRadians(180))), drivetrain));
        }


        // MARK: Button H
        Trigger positionHTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.8 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.8
        );
        if(ally.isPresent()){
            if(ally.get() == Alliance.Blue){
                positionHTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(5.782, 4.360, new Rotation2d(Units.degreesToRadians(180))), drivetrain));
            }
            if(ally.get() == Alliance.Red){
                positionHTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(14.338, 4.366, new Rotation2d(Units.degreesToRadians(180))), drivetrain));
            }
        }
        else{
            positionHTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(5.782, 4.360, new Rotation2d(Units.degreesToRadians(180))), drivetrain));
        }


        // MARK: Button I
        Trigger positionITrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == 0.9 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == 0.9
        );
        if(ally.isPresent()){
            if(ally.get() == Alliance.Blue){
                positionITrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(5.056, 5.154, new Rotation2d(Units.degreesToRadians(240))), drivetrain));
            }
            if(ally.get() == Alliance.Red){
                positionITrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(13.708, 5.129, new Rotation2d(Units.degreesToRadians(240))), drivetrain));
            }
        }
        else{
            positionITrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(5.056, 5.154, new Rotation2d(Units.degreesToRadians(240))), drivetrain));
        }


        // MARK: Button J
        Trigger positionJTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == -0.1 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == -0.9
        );
        if(ally.isPresent()){
            if(ally.get() == Alliance.Blue){
                positionJTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(4.788, 5.318, new Rotation2d(Units.degreesToRadians(240))), drivetrain));
            }
            if(ally.get() == Alliance.Red){
                positionJTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(13.430, 5.294, new Rotation2d(Units.degreesToRadians(240))), drivetrain));
            }
        }
        else{
            positionJTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(4.788, 5.318, new Rotation2d(Units.degreesToRadians(240))), drivetrain));
        }


        // MARK: Button K
        Trigger positionKTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == -0.2 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == -0.8
        );
        if(ally.isPresent()){
            if(ally.get() == Alliance.Blue){
                positionKTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(3.869, 5.167, new Rotation2d(Units.degreesToRadians(300))), drivetrain));
            }
            if(ally.get() == Alliance.Red){
                positionKTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(12.449, 5.140, new Rotation2d(Units.degreesToRadians(300))), drivetrain));
            }
        }
        else{
            positionKTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(3.869, 5.167, new Rotation2d(Units.degreesToRadians(300))), drivetrain));
        }


        // MARK: Button L
        Trigger positionLTrigger = new Trigger(() ->
            Math.round(joystick.getLeftX() * 10) / 10.0 == -0.3 &&
            Math.round(joystick.getLeftY() * 10) / 10.0 == -0.7
        );
        if(ally.isPresent()){
            if(ally.get() == Alliance.Blue){
                positionLTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(3.532, 4.964, new Rotation2d(Units.degreesToRadians(300))), drivetrain));
            }
            if(ally.get() == Alliance.Red){
                positionLTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(12.119, 4.964, new Rotation2d(Units.degreesToRadians(300))), drivetrain));
            }
        }
        else{
            positionLTrigger.onTrue(new AlignOnTheFlyByPose(new Pose2d(3.532, 4.964, new Rotation2d(Units.degreesToRadians(300))), drivetrain));
        }


        // MARK: DPAD Bindings
        joystick.povDown().onTrue(new LevelManager(Levels.LEVEL_1, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povLeft().onTrue(new LevelManager(Levels.LEVEL_2, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povRight().onTrue(new LevelManager(Levels.LEVEL_3, elevatorSubsystem, coralSubsystem).goToPreset());
        joystick.povUp().onTrue(new LevelManager(Levels.LEVEL_4, elevatorSubsystem, coralSubsystem).goToPreset());
    }    
}
