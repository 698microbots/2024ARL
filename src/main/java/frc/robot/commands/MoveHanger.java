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
  private boolean reverse;
  private boolean both;7

  /** Creates a new AutoHanger. */
  public MoveHanger(boolean reverse, boolean leftMotor, HangerSubsystem hangerSubsystem) {
    this.reverse = reverse;
    this.hangerSubsystem = hangerSubsystem;
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
    if (!reverse){
      if (leftMotor) {
        hangerSubsystem.setHangerMotorOne(.6);
        System.out.println("left forward");
      } else {
        hangerSubsystem.setHangerMotorTwo(.6);
        System.out.println("right forward");
      }
    } else {
      if (leftMotor){
        hangerSubsystem.setHangerMotorOne(-.6);
        System.out.println("left reverse");
      }else {
        hangerSubsystem.setHangerMotorTwo(-.6);
        System.out.println("right reverse");

      }
    }
    // hangerSubsystem.setHangerMotorOne(.5);
    // hangerSubsystem.setHangerMotorTwo(.5);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hangerSubsystem.setHangerMotorOne(0);
    hangerSubsystem.setHangerMotorTwo(0);
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
