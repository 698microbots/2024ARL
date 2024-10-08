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
import frc.robot.subsystems.ArmSubsystem;
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
    // m_robotContainer.field2d.setRobotPose(m_robotContainer.pose); 
    // System.out.println("STATES:/n" + m_robotContainer.drivetrain.getSwerveModTarget());
    SmartDashboard.putNumber("Left Y", m_robotContainer.joystick.getLeftY());
    SmartDashboard.putNumber("Left X", m_robotContainer.joystick.getLeftX());


    SmartDashboard.putNumber("Left2 Y", m_robotContainer.joystick2.getLeftY());
    SmartDashboard.putNumber("Left2 X", m_robotContainer.joystick2.getLeftX());
    SmartDashboard.putNumber("Robot Y", m_robotContainer.drivetrain.getState().Pose.getY()); // says this is null when
                                                                                             // simulating
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
 
    SmartDashboard.putNumber("FL DRIVE Velocity", m_robotContainer.driveTrainVoltages.FLDVoltage());
    SmartDashboard.putNumber("BL DRIVE Velocity", m_robotContainer.driveTrainVoltages.BLDVoltage());
    SmartDashboard.putNumber("FR DRIVE Velocity", m_robotContainer.driveTrainVoltages.FRDVoltage());
    SmartDashboard.putNumber("BR DRIVE Velocity", m_robotContainer.driveTrainVoltages.BRDVoltage());

    SmartDashboard.putNumber("Robot Pose to amp X: ", m_robotContainer.limeLight.getRelative3dBotPose().getX());
    SmartDashboard.putNumber("Robot Pose to amp Y:", m_robotContainer.limeLight.getRelative3dBotPose().getY());
    SmartDashboard.putNumber("Robot Pose to amp Z:", m_robotContainer.limeLight.getRelative3dBotPose().getZ());
    
    // SmartDashboard.putNumber("Target Space X", m_robotContainer.limeLight.getRobotPoseX());
    // SmartDashboard.putNumber("Target Pose Y", m_robotContainer.limeLight.getRobotPoseZ());
    // SmartDashboard.putData("Field2d", m_robotContainer.field2d);
    // SmartDashboard.putNumber("Intake Volts: ", m_robotContainer.intake.getIntakeVolts());

    SmartDashboard.putBoolean("isBlocked", m_robotContainer.intake.getBlocked());
    SmartDashboard.putBoolean("isBlocked2", m_robotContainer.intake.getBlocked2());
    
    SmartDashboard.putNumber("Note Angle", m_robotContainer.limeLight.getNoteHorizontalAngle());
    SmartDashboard.putNumber("Note Area", m_robotContainer.limeLight.getNoteArea());
    
    SmartDashboard.putNumber("horizontal angle", m_robotContainer.limeLight.getH_angle());
    SmartDashboard.putNumber("arm encoder", m_robotContainer.arm.getEncoder());
 // says this is null when
                                                                                             // simulating
    SmartDashboard.putNumber("distance to speaker", m_robotContainer.limeLight.getDistToSpeaker());
    SmartDashboard.putNumber("v angle", m_robotContainer.limeLight.getV_angle());;//just giving directly from limelight
  
    SmartDashboard.putNumber("Slew Rate X", m_robotContainer.slewRateDriveX.calculate(-m_robotContainer.joystick.getLeftY()));
    SmartDashboard.putNumber("Slew Rate Y", m_robotContainer.slewRateDriveY.calculate(-m_robotContainer.joystick.getLeftX()));
    SmartDashboard.putNumber("Slew Rate Turn", m_robotContainer.slewRateTurn.calculate(-m_robotContainer.joystick.getRightX()));
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

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
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
