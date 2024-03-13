// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class TESTAutoArm extends Command {
  /** Creates a new TESTAutoArm. */
  private final ArmSubsystem armSubsystem;
  private final PIDController pidController = new PIDController(1.1, 0, 0.0);
  public TESTAutoArm(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.\
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(armSubsystem.getEncoder(), .34);
    System.out.println("arm encoder: " + armSubsystem.getEncoder());
    armSubsystem.moveArm(-speed);
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
