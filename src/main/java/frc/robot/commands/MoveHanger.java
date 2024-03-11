// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HangerSubsystem;

public class MoveHanger extends Command {
  // instance variables
  private HangerSubsystem hangerSubsystem;
  private boolean leftMotor;
  private boolean HangerDown;
  private boolean both;

  /** Creates a new AutoHanger. */
  public MoveHanger(boolean both, boolean HangerDown, boolean leftMotor, HangerSubsystem hangerSubsystem) {
    // this.HangerDown = HangerDown;
    this.leftMotor = leftMotor;
    this.both = both;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hangerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // TODO - find directionality of the motors
    if (leftMotor) {
      hangerSubsystem.setHangerMotorOne(.1);
      // if (HangerDown) {
      //   double time = 5; // the time it takes for the Hanger to go down
      //   double rate = .1 / time;
      //   for (var i = .1; i >= (0 - rate); i -= rate) {
      //     // System.out.println(Math.round(i * 100.0) / 100.0);
      //     hangerSubsystem.setHangerMotorOne(rate);
          
      //   }
      // }
    } else if (both) {
      hangerSubsystem.setHangerMotorOne(.1);
      hangerSubsystem.setHangerMotorTwo(.1);
      // if (HangerDown) {
      //   double time = 5; // the time it takes for the Hanger to go down
      //   double rate = .1 / time;
      //   for (var i = .1; i >= (0 - rate); i -= rate) {
      //     System.out.println(Math.round(i * 100.0) / 100.0);
      //   }
      // }
    } else {
      hangerSubsystem.setHangerMotorTwo(.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
