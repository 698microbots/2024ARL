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
import frc.robot.subsystems.LimeLightHelpersSubsystem.LimelightResults;

public class AutoCenterNote extends Command {
  /** Creates a new AutoCenterNote. */
private double angle;
private PIDController pidController = new PIDController(.04, 0, 0.001); //kp as 0.05 works, everything else as 0
//dont use I for pid
private LimeLightSubsystem limeLightSubsystem;
private CommandSwerveDrivetrain drivetrain;
private final SwerveRequest.FieldCentric swerveCentric = new SwerveRequest.FieldCentric(); //might change this to swerve centric
private double maxRotationSpeed = 3;
private Supplier<Double> ySpeed, xSpeed;
  public AutoCenterNote(Supplier<Double> ySpeed, Supplier<Double> xSpeed, CommandSwerveDrivetrain drivetrain, LimeLightSubsystem limeLightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.limeLightSubsystem = limeLightSubsystem;
    this.ySpeed = ySpeed;
    this.xSpeed = xSpeed;
    addRequirements(drivetrain);
    addRequirements(limeLightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = xSpeed.get();
    double y = ySpeed.get();

    // System.out.println(limeLightSubsystem.getNoteHorizontalAngle());
    // System.out.println("THIS IS WORKING");
 
    
    angle = limeLightSubsystem.getNoteHorizontalAngle(); 
    double rotationSpeed = pidController.calculate(angle,0);
    if (Math.abs(maxRotationSpeed) > 1 ){
      maxRotationSpeed = 1 * Math.signum(maxRotationSpeed);
    }


    // System.out.println("Rotation Speed: " + rotationSpeed);
    // System.out.println("Angle: " + angle);
    drivetrain.setControl(swerveCentric.withVelocityX(-x).withVelocityY(-y).withRotationalRate(rotationSpeed));
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
