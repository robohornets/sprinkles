// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helpers.ShuffleboardUtil;
import frc.robot.subsystems.mechanisms.algae.AlgaeSubsystem;
import frc.robot.subsystems.mechanisms.coral.CoralAngleManager;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;
import frc.robot.subsystems.photonvision.Vision;
import frc.robot.subsystems.photonvision.VisionSubsystem;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private Vision vision;

  public CANrange canRangeSensor = new CANrange(34);

  private final boolean kUseLimelight = false;

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
    AlgaeSubsystem.angleAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);
    AlgaeSubsystem.flywheelAlgaeMotor.setNeutralMode(NeutralModeValue.Brake);

    CoralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
    CoralSubsystem.flywheelMotor.setNeutralMode(NeutralModeValue.Brake);

    ElevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
    ElevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
  }

  private double lastPrintTime = 0;
  @Override
  public void robotPeriodic() {
    pdp.clearStickyFaults();

    if (m_robotContainer.elevatorSubsystem.elevatorLeft.get() < -50) {
      m_robotContainer.slowRobotSpeed = true;
    }
    else {
      m_robotContainer.slowRobotSpeed = false;
    }

    CommandScheduler.getInstance().run();

    //System.out.println(CoralVariables.angleMotor.getPosition().getValueAsDouble());

    ShuffleboardUtil.put("Cameras Enabled", m_robotContainer.camerasEnabled);
    ShuffleboardUtil.put("Elevator Height", ElevatorSubsystem.elevatorLeft.getPosition().getValueAsDouble());
    //ShuffleboardUtil.put("Angle Encoder is Connected", CoralSubsystem.angleDCEncoder.isConnected());
    ShuffleboardUtil.put("Coral Angle", CoralSubsystem.angleDCEncoder.get());
    ShuffleboardUtil.put("Robot Pose", RobotContainer.drivetrain.getState().Pose);
    //ShuffleboardUtil.put("Angle Motor Position", Math.round(CoralSubsystem.angleDCEncoder.get() * 10)/10);

    ShuffleboardUtil.put("Algae Angle", AlgaeSubsystem.angleAlgaeDCEncoder.get());

    // print(RobotContainer.disableControllerIn);
    // Test code for CANrange sensor
    // System.out.println(canRangeSensor.getDistance(true).refresh().getValueAsDouble());
    // System.out.println(VisionSubsystem.getLatestEstimatedPose());
  }

  @Override
  public void disabledInit() {
    ElevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Coast);
    ElevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Coast);
    CoralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_robotContainer.drivetrain.resetPose(new PathPlannerAuto(m_autonomousCommand.getName()).getStartingPose());
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

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

    if (m_robotContainer.camerasEnabled) {
      // Get current time in seconds
      double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

      if (currentTime - lastPrintTime >= 2.0) { 
          lastPrintTime = currentTime;  // Reset timer

          // Print the current odometry pose
          System.out.println("[Odometry] Current Pose: " + RobotContainer.drivetrain.getState().Pose);

          // Check for vision estimate
          var visionEst = vision.getEstimatedGlobalPose();
          visionEst.ifPresent(est -> {
              Pose2d estimatedPose = est.estimatedPose.toPose2d();

              Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;

              Pose2d newPose = new Pose2d(
                estimatedPose.getTranslation(),
                currentPose.getRotation()
              );

              var estStdDevs = vision.getEstimationStdDevs();
              System.out.println("[Vision] Estimated Pose: " + newPose);
              ShuffleboardUtil.put("Vision Estimated Pose", newPose);
              m_robotContainer.drivetrain.resetPose(newPose);
              //m_robotContainer.drivetrain.addVisionMeasurement(estimatedPose, currentTime, estStdDevs);
          });
      }
    }
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
