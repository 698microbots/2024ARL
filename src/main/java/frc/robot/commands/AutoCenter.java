// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.LimeLightSubsystem;

public class AutoCenter extends Command {
// Instance Variables


private PIDController pidController = new PIDController(.5, 0, 0);
private LimeLightSubsystem limeLightSubsystem;
private CommandSwerveDrivetrain drivetrain;
private boolean end = false;
private final SwerveRequest.RobotCentric swerveCentric = new SwerveRequest.RobotCentric();

  /** Creates a new AutoCenter. */
  public AutoCenter(CommandSwerveDrivetrain drivetrain, LimeLightSubsystem limeLightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLightSubsystem = limeLightSubsystem;
    this.drivetrain = drivetrain;
    addRequirements(limeLightSubsystem);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(limeLightSubsystem.getH_angle(),0);
    if (speed > 2.5 ){
      speed = 2.5;
    }

    

    drivetrain.setControl(swerveCentric.withRotationalRate(speed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (end){
      end = true;
      return true;
    } else {
      return false;
    }
  }
}
