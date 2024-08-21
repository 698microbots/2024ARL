// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class AUTOTESTarmDown extends Command {
  /** Creates a new AUTOTESTarmDown. */
  private final ArmSubsystem armSubsystem;
  private int counter = 0;
  public AUTOTESTarmDown(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    armSubsystem.moveArm(.30);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.moveArm(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter > Constants.numSeconds(.3)){
      return true;
    } else {
      return false;
    }
  }
}
