// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoTrap extends Command {
  /** Creates a new AutoTrap. */
  private final PIDController pidController = new PIDController(.95, 0, 0);
  private FlywheelSubsystem flywheelSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ArmSubsystem armSubsystem;
  private int counter = 0;
  public AutoTrap(FlywheelSubsystem flywheelSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.flywheelSubsystem = flywheelSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.armSubsystem = armSubsystem;
    addRequirements(flywheelSubsystem);
    addRequirements(intakeSubsystem);
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    
    armSubsystem.moveArm(-pidController.calculate(armSubsystem.getEncoder(), Constants.encoderTrap)); //oriignal was .308

    if (counter > Constants.numSeconds(1)){
      flywheelSubsystem.setFlywheelMotorSpeed(1);
    }

    if (counter > Constants.numSeconds(1.5)){
      intakeSubsystem.backupIntakeMotor(.9);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.moveArm(0);
    intakeSubsystem.backupIntakeMotor(0);
    flywheelSubsystem.setFlywheelMotorSpeed(0);
    counter = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
