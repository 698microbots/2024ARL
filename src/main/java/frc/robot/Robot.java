// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.System.Logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LimeLightHelpersSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    // m_robotContainer.limeLight.setLight(false); limelight auto turns off this
    DataLogManager.start();
    //TODO: Get Advantagescope logging to work, already able to set the ip for it but its still not connecting
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    // System.out.println("STATES:/n" + m_robotContainer.drivetrain.getSwerveModTarget());
    SmartDashboard.putNumber("Left Y", m_robotContainer.joystick.getLeftY());
    SmartDashboard.putNumber("Left X", m_robotContainer.joystick.getLeftX());

    SmartDashboard.putNumber("Robot Y", m_robotContainer.drivetrain.getState().Pose.getY()); //says this is null when simulating
    SmartDashboard.putNumber("Robot X", m_robotContainer.drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("AprilTag ID", LimeLightHelpersSubsystem.getFiducialID("limelight"));
    SmartDashboard.putNumber("H Angle", m_robotContainer.limeLight.getH_angle());
    SmartDashboard.putNumber("Angle", m_robotContainer.gyro.getAngle());
    SmartDashboard.putString("Target States", m_robotContainer.drivetrain.getSwerveModTarget());
    // SmartDashboard.putNumber("Rotation Mod 1", kDefaultPeriod);
    // m_robotContainer.logger.telemeterize(m_robotContainer.drivetrain.getState());
    //TODO: Calculate the rate of change of the positions of the motors (NOT VOLTAGES RN ITS POSITION)
    SmartDashboard.putNumber("FL DRIVE Voltage", m_robotContainer.driveTrainVoltages.FLDVoltage());
    SmartDashboard.putNumber("BL DRIVE Voltage", m_robotContainer.driveTrainVoltages.BLDVoltage());
    SmartDashboard.putNumber("FR DRIVE Voltage", m_robotContainer.driveTrainVoltages.FRDVoltage());
    SmartDashboard.putNumber("BR DRIVE Voltage", m_robotContainer.driveTrainVoltages.BRDVoltage());

    SmartDashboard.putNumber("FL TURN Voltage", m_robotContainer.driveTrainVoltages.FLTVoltage());
    SmartDashboard.putNumber("BL TURN Voltage", m_robotContainer.driveTrainVoltages.BLTVoltage());
    SmartDashboard.putNumber("FR TURN Voltage", m_robotContainer.driveTrainVoltages.FRTVoltage());
    SmartDashboard.putNumber("BR TURN Voltage", m_robotContainer.driveTrainVoltages.BRTVoltage());
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
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
    // System.out.println("running teleop periodic");

    
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
