// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class SetFlywheelMotor extends Command {
  FlywheelSubsystem flywheel = new FlywheelSubsystem(); // a new Flywheel object
  /** Creates a new moveFlywheelMotor. */
  public SetFlywheelMotor() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  // make the motor start moving
  @Override
  public void initialize() {
    flywheel.setFlywheelMotorSpeed(.05);
  }

  // Called every time the scheduler runs while the command is scheduled.
  // wait for 10 seconds
  @Override
  public void execute() {
    try {
      wait(10000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  // Called once the command ends or is interrupted.
  // stop the motor
  @Override
  public void end(boolean interrupted) {
    flywheel.setFlywheelMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
