// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class TESTFlywheel extends Command {
  /** Creates a new TESTFlywheel. */
  private final FlywheelSubsystem flywheelSubsystem;
  private Supplier<Double> ySpeed;
  public TESTFlywheel(FlywheelSubsystem flywheelSubsystem, Supplier<Double> ySpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ySpeed = ySpeed;
    this.flywheelSubsystem = flywheelSubsystem;
    addRequirements(flywheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = ySpeed.get();

    if (speed > 0){
    flywheelSubsystem.setFlywheelMotorSpeed(-speed);
    } else {
    flywheelSubsystem.setFlywheelMotorSpeed(-.05);
    }
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
