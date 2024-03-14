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

public class AutoCenterAmp extends Command {
  /** Creates a new AutoCenterAmp. */
  private double angle;
  private PIDController pidControllerCenter = new PIDController(.04, 0.01, 0.0); //kp as 0.05 works, everything else as 0
  private Supplier<Double> xSpeed, ySpeed;
  private double maxRotationSpeed;
  private boolean end = false;
  private CommandSwerveDrivetrain drivetrain;
  private LimeLightSubsystem limeLightSubsystem;
  private final SwerveRequest.FieldCentric swerveCentric = new SwerveRequest.FieldCentric(); //might change this to swerve centric

  public AutoCenterAmp(
    CommandSwerveDrivetrain drivetrain,
    Supplier<Double> xSpeed,
    Supplier<Double> ySpeed,
    double maxRotationSpeed,
    LimeLightSubsystem limeLightSubsystem
  ) {
    this.drivetrain = drivetrain;
    this.limeLightSubsystem = limeLightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.maxRotationSpeed = maxRotationSpeed;
    addRequirements(drivetrain);
    addRequirements(limeLightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = xSpeed.get();
    double y = ySpeed.get();

    angle = limeLightSubsystem.getH_angle(); 
    double speed = pidControllerCenter.calculate(angle,0);
    if (Math.abs(maxRotationSpeed) > 1 ){
      maxRotationSpeed = 1 * Math.signum(maxRotationSpeed);
    }

    if (angle <=1 ){
      end = true;
    }

    System.out.println("Rotation Speed: " + speed);
    System.out.println("Angle: " + angle);
    drivetrain.setControl(swerveCentric.withVelocityX(-x).withVelocityY(-y).withRotationalRate(speed));    
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
