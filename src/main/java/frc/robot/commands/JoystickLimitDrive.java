// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.driveTrainVoltages;

public class JoystickLimitDrive extends Command {

  private CommandSwerveDrivetrain commandSwerveDrivetrain;

  private Supplier<Double> x, y, theta;
  private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(.5);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  private double MaxSpeed, MaxAngularRate;
  /** Creates a new JoystickLimitDrive. */
  public JoystickLimitDrive(CommandSwerveDrivetrain commandSwerveDrivetrain, Supplier<Double> x, Supplier<Double> y,
      Supplier<Double> theta, double MaxSpeed, double MaxAngularRate) {
    this.theta = theta;
    this.x = x;
    this.y = y;
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.MaxAngularRate = MaxAngularRate;
    this.MaxSpeed = MaxSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(commandSwerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double slewX = slewRateLimiter.calculate(x.get());
    double slewY = slewRateLimiter.calculate(y.get());
    double slewTheta = slewRateLimiter.calculate(theta.get());
    commandSwerveDrivetrain.setControl(drive.withVelocityX(-slewX * MaxSpeed).withVelocityY(-slewY * MaxSpeed).withRotationalRate(-slewTheta * MaxAngularRate));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
       commandSwerveDrivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
