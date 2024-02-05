// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelSetIdle extends Command {
  private final FlywheelSubsystem flywheelSubsystem; // a new Flywheel object
  /** Creates a new moveFlywheelMotor. */
  public FlywheelSetIdle(FlywheelSubsystem flywheelSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.flywheelSubsystem = flywheelSubsystem;
    addRequirements(flywheelSubsystem);
  }

  // Called when the command is initially scheduled.
  // make the motor start moving
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // wait for 10 seconds
  @Override
  public void execute() {
    flywheelSubsystem.setFlywheelMotorSpeed(.05);
  }

  // Called once the command ends or is interrupted.
  // stop the motor
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
