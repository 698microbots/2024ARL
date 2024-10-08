// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class TESTMoveArm extends Command {
  /** Creates a new MoveArm. */
  private final ArmSubsystem armSubsystem; 
  private Supplier<Double> ySpeed;
  private double speed = 0;
  public TESTMoveArm(ArmSubsystem armSubsystem, Supplier<Double> ySpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.ySpeed = ySpeed;
    addRequirements(armSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = ySpeed.get();
    armSubsystem.moveArm(speed * .3);
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
