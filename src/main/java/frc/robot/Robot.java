// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helpers.ShuffleboardUtil;
import frc.robot.subsystems.mechanisms.coral.CoralAngleManager;
import frc.robot.subsystems.mechanisms.coral.CoralSubsystem;
import frc.robot.subsystems.mechanisms.elevator.ElevatorSubsystem;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public CANrange canRangeSensor = new CANrange(34);

  private final boolean kUseLimelight = false;

  private PowerDistribution pdp = new PowerDistribution();

  //public static Encoder elevatorEncoder = new Encoder(0, 1);

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    //elevatorEncoder.setDistancePerPulse(1.0/2048);
    CoralSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
    ElevatorSubsystem.elevatorLeft.setNeutralMode(NeutralModeValue.Brake);
    ElevatorSubsystem.elevatorRight.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void robotPeriodic() {
    pdp.clearStickyFaults();
    
    CommandScheduler.getInstance().run();

    m_robotContainer.updateVisionOdometry();

    //System.out.println(CoralVariables.angleMotor.getPosition().getValueAsDouble());

    ShuffleboardUtil.put("Elevator Height", ElevatorSubsystem.elevatorLeft.getPosition().getValueAsDouble());
    ShuffleboardUtil.put("Angle Encoder is Connected", CoralSubsystem.angleDCEncoder.isConnected());
    ShuffleboardUtil.put("Coral Angle", CoralSubsystem.angleDCEncoder.get());
    ShuffleboardUtil.put("Robot Pose", RobotContainer.drivetrain.getState().Pose);
    ShuffleboardUtil.put("Angle Motor Position", Math.round(CoralSubsystem.angleDCEncoder.get() * 10)/10);
    
    // System.out.println(ElevatorVariables.elevatorLeft.getPosition().getValueAsDouble());
    // System.out.println(CoralVariables.angleDCEncoder.get());
    //double num = m_robotContainer.encoder1.get();
    //System.out.println(num);

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    

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
