// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.constant.Constable;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.LimeLightSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;

public class AutoPosition extends Command {
  // instance variables
  private double hypot = 0;
  private double turnAngle = 0;
  private double orientationAngle = 90 - turnAngle;
  private double zDisp = 0; 
  private double xDisp = 0;
  private LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();
  private final SwerveRequest.RobotCentric swerveCentric = new SwerveRequest.RobotCentric();
  private CommandSwerveDrivetrain drivetrain;
  private PIDController pidController = new PIDController(Constants.kp, Constants.ki, Constants.kd);
  

  /** Creates a new AutoPosition. */
  public AutoPosition(LimeLightSubsystem limeLightSubsystem, CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLightSubsystem = limeLightSubsystem;
    this.drivetrain = drivetrain;
    addRequirements(limeLightSubsystem);
    addRequirements(drivetrain);
    hypot = limeLightSubsystem.calcHypotenuse();
    turnAngle = limeLightSubsystem.getH_angle();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    zDisp = Math.hypot(limeLightSubsystem.getXDist(), limeLightSubsystem.getYDist());
    drivetrain.setControl(swerveCentric.withVelocityX(limeLightSubsystem.getXDist()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
