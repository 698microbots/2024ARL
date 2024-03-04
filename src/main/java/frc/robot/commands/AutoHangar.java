// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HangerSubsystem;

public class AutoHangar extends Command {
  // instance variables
  private HangerSubsystem hangerSubsystem;
  private boolean leftMotor;
  private boolean hangarDown;
  /** Creates a new AutoHangar. */
  public AutoHangar(boolean hangarDown, boolean leftMotor, HangerSubsystem hangerSubsystem) { // TODO - add a boolean for whether we want to raise hangar or not
    this.hangarDown = hangarDown;
    this.leftMotor = leftMotor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hangerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  // TODO - Use for loop to slow motor speed for raising hangar
  public void execute() { // TODO - find directionality of the motors
    if (leftMotor) {
      hangerSubsystem.setHangarMotorOne(.1);
      if (hangarDown) {
        // TODO - add a for loop to decrease speed
      }
    } else {
      hangerSubsystem.setHangarMotorTwo(.1);
      if (hangarDown) {
        // TODO - add a for loop to decrease speed
      }
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
