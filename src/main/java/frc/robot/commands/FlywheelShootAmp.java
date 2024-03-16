// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class FlywheelShootAmp extends Command {
  private final FlywheelSubsystem flywheelSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  /** Creates a new FlywheelShootAmp. */
  public FlywheelShootAmp(FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.flywheelSubsystem = flywheelSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(flywheelSubsystem);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.backupIntakeMotor(.75);
    flywheelSubsystem.setFlywheelMotorSpeed(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.backupIntakeMotor(0);
    flywheelSubsystem.stopFlywheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
