// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.invoke.ConstantBootstraps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMove extends Command {
  /** Creates a new IntakeMove. */
  private final IntakeSubsystem intakeSubsystem;
  private final boolean yes;
  private int counter = 0;
  public IntakeMove(IntakeSubsystem intakeSubsystem, boolean yes) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.yes = yes;
    addRequirements(intakeSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (yes){
      intakeSubsystem.setIntakeMotor(-.5);
    } else {
      intakeSubsystem.setIntakeMotor(0);
      
    }
    System.out.println("INTAKE RUNNING");

    // if (intakeSubsystem.getCanRun()){
    //   intakeSubsystem.setIntakeMotor(-.75);
    //   counter = 0;
    // } else {
    //   intakeSubsystem.setIntakeMotor(0);
    // }

    // if (intakeSubsystem.getIntakeVolts() > Constants.intakeNoteVoltage){
    //   counter++;
    // }

    // if (counter > Constants.numSeconds(2)){
    //   intakeSubsystem.setCanRun(false);
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
