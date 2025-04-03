// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.helpers.ShuffleboardUtil;
import frc.robot.subsystems.photonvision.Vision;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private Vision vision;

  private PowerDistribution pdp = new PowerDistribution();
  

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    // MARK: Configure Cameras
    // UsbCamera zane1 = CameraServer.startAutomaticCapture(0);
    // zane1.setResolution(320, 180);
    // zane1.setFPS(60);

    // vision = new Vision();

    // MARK: Set Brake Modes
    m_robotContainer.coralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.coralSubsystem.flywheelMotor.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.coralSubsystem.funnelLeft.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.coralSubsystem.funnelRight.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.algaeSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.elevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.elevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.coralSubsystem.funnelLeft.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.coralSubsystem.funnelRight.setNeutralMode(NeutralModeValue.Brake);

    CommandScheduler.getInstance().cancelAll();
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

    // MARK: Put Shuffleboard Values
    ShuffleboardUtil.put("Time Remaining", DriverStation.getMatchTime());
    ShuffleboardUtil.put("Funnel CANRange Distance", m_robotContainer.coralSubsystem.funnelSensor.getDistance(true).getValueAsDouble());
    ShuffleboardUtil.put("Funnel CANRange Bool", m_robotContainer.coralSubsystem.funnelTrigger.getAsBoolean());

    ShuffleboardUtil.put("Elevator Height", m_robotContainer.elevatorSubsystem.getElevatorHeight());
    
    ShuffleboardUtil.put("Algae Angle", m_robotContainer.algaeSubsystem.angleDCEncoder.get());

    ShuffleboardUtil.put("Coral CANRange Distance", m_robotContainer.coralSubsystem.coralForwardSensor.getDistance(true).getValueAsDouble());
    ShuffleboardUtil.put("Coral CANRange Bool", m_robotContainer.coralSubsystem.coralForwardTrigger.getAsBoolean());

    ShuffleboardUtil.put("Coral Angle", m_robotContainer.coralSubsystem.angleDCEncoder.get());
    ShuffleboardUtil.put("Coral Angle Kraken", m_robotContainer.coralSubsystem.angleMotor.getPosition().getValueAsDouble());
    if (DriverStation.getMatchTime() <= 1.5) {
      m_robotContainer.coralSubsystem.flywheelOut();
    }
  }

  @Override
  public void disabledInit() {
    m_robotContainer.coralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.algaeSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.coralSubsystem.funnelLeft.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.coralSubsystem.funnelRight.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.coralSubsystem.flywheelMotor.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.coralSubsystem.funnelLeft.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.coralSubsystem.funnelRight.setNeutralMode(NeutralModeValue.Brake);

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
}
