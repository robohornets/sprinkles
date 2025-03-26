// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helpers.ShuffleboardUtil;
import frc.robot.subsystems.photonvision.Vision;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private Vision vision;

  public CANrange canRangeSensor = new CANrange(34);

  private PowerDistribution pdp = new PowerDistribution();

  //public static Encoder elevatorEncoder = new Encoder(0, 1);

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    UsbCamera zane1 = CameraServer.startAutomaticCapture(0);
    UsbCamera zane2 = CameraServer.startAutomaticCapture(1);
    zane1.setResolution(320, 180);
    zane1.setFPS(60);
    zane2.setResolution(640, 360);
    zane2.setFPS(30);


    vision = new Vision();
    m_robotContainer.algaeSubsystem.angleAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.algaeSubsystem.flywheelAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);

    m_robotContainer.coralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.coralSubsystem.flywheelMotor.setNeutralMode(NeutralModeValue.Brake);

    m_robotContainer.elevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.elevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void robotPeriodic() {
    pdp.clearStickyFaults();

    if (m_robotContainer.elevatorSubsystem.elevatorLeft.getPosition().getValueAsDouble() <= -50) {
      m_robotContainer.slowRobotSpeed = true;
    }
    else {
      m_robotContainer.slowRobotSpeed = false;
    }

    CommandScheduler.getInstance().run();

    ShuffleboardUtil.put("Time Remaining", DriverStation.getMatchTime());
    ShuffleboardUtil.put("Canrange", m_robotContainer.canRangeSensor.getDistance(true).getValueAsDouble());
    ShuffleboardUtil.put("elevator offset", m_robotContainer.elevatorSubsystem.elevatorEncoderOffset);
    // ShuffleboardUtil.put("Canrange senses", m_robotContainer.canRangeTrigger().ge);
    ShuffleboardUtil.put("Slow Robot Speed", m_robotContainer.slowRobotSpeed);
    ShuffleboardUtil.put("canrange connected", m_robotContainer.canRangeSensor.isConnected());
    ShuffleboardUtil.put("Elevator Height", m_robotContainer.elevatorSubsystem.getElevatorHeight());
    ShuffleboardUtil.put("Coral Angle", m_robotContainer.coralSubsystem.angleDCEncoder.get());
    ShuffleboardUtil.put("kraken Coral Angle", m_robotContainer.coralSubsystem.angleMotor.getPosition().getValueAsDouble());
    ShuffleboardUtil.put("Robot Pose", RobotContainer.drivetrain.getState().Pose);
    ShuffleboardUtil.put("Algae Angle", m_robotContainer.algaeSubsystem.angleAlgaeMotor.getPosition().getValueAsDouble());

    if (DriverStation.getMatchTime() <= 1.5) {
      m_robotContainer.coralSubsystem.flywheelOut();
    }
  }

  @Override
  public void disabledInit() {
    m_robotContainer.coralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);

    Timer.delay(5);
    m_robotContainer.elevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Coast);
    m_robotContainer.elevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      // if (m_autonomousCommand.getName() != null || new PathPlannerAuto(m_autonomousCommand.getName()).getStartingPose() != null) {
      //   m_robotContainer.drivetrain.resetPose(new PathPlannerAuto(m_autonomousCommand.getName()).getStartingPose());
      // }
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    //System.out.println("------");
    //System.out.println(elevatorEncoder.get());
    //System.out.println(elevatorEncoder.getDistance());

    // if (m_robotContainer.camerasEnabled) {
    //   // Get current time in seconds
    //   double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    //   if (currentTime - lastPrintTime >= 2.0) { 
    //       lastPrintTime = currentTime;  // Reset timer

    //       // Print the current odometry pose
    //       System.out.println("[Odometry] Current Pose: " + RobotContainer.drivetrain.getState().Pose);

    //       // Check for vision estimate
    //       var visionEst = vision.getEstimatedGlobalPose();
    //       visionEst.ifPresent(est -> {
    //           Pose2d estimatedPose = est.estimatedPose.toPose2d();

    //           Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;

    //           Pose2d newPose = new Pose2d(
    //             estimatedPose.getTranslation(),
    //             currentPose.getRotation()
    //           );

    //           var estStdDevs = vision.getEstimationStdDevs();
    //           System.out.println("[Vision] Estimated Pose: " + newPose);
    //           ShuffleboardUtil.put("Vision Estimated Pose", newPose);
    //           m_robotContainer.drivetrain.resetPose(newPose);
    //           //m_robotContainer.drivetrain.addVisionMeasurement(estimatedPose, currentTime, estStdDevs);
    //       });
    //   }
    // }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}


  // This is because I am too lazy to type sysout all the time
  public void print(Object printValue) {
    System.out.println(printValue);
  }
}
