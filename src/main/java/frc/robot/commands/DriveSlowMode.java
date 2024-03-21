// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLightSubsystem;

public class DriveSlowMode extends Command {
  private PIDController pidControllerCenter = new PIDController(.04, 0, 0.001); // kp as 0.05 works, everything else as 0
  private CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric swerveCentric = new SwerveRequest.FieldCentric(); // might change this to swerve centric
  private Supplier<Double> xSpeed, ySpeed, rotationRate;

  /** Creates a new SlowMode. */
  public DriveSlowMode(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotationRate) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotationRate = rotationRate;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = xSpeed.get() * .5;
    double y = ySpeed.get() * .5;
    double rotateRate = rotationRate.get() * .5;
    drivetrain.setControl(swerveCentric.withVelocityX(x).withVelocityY(y).withRotationalRate(rotateRate));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(swerveCentric.withVelocityX(0).withVelocityY(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
