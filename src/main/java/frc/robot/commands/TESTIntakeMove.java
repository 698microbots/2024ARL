// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class TESTIntakeMove extends Command {
  /** Creates a new TESTIntakeMove. */
  private final IntakeSubsystem intakeSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;
  private double counter = 0;
  public TESTIntakeMove(IntakeSubsystem intakeSubsystem, FlywheelSubsystem flywheelSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.flywheelSubsystem = flywheelSubsystem;
    addRequirements(intakeSubsystem);
    addRequirements(flywheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.reverseIntakeMotor(-.75);
    System.out.println("reversed intake");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.reverseIntakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
