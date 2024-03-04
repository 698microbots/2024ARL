// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class IntakeMove extends Command { // TODO - add CANdle (led strips) functionality
  /** Creates a new IntakeMove. */
  private final IntakeSubsystem intakeSubsystem;
  private final LimeLightSubsystem limelight;
  // private  boolean yes = false;
  private int counter = 0;
  public IntakeMove(IntakeSubsystem intakeSubsystem, LimeLightSubsystem limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.limelight = limelight;
    // this.yes = yes;
    addRequirements(intakeSubsystem);
    addRequirements(limelight);
    
  }

  // public IntakeMove(IntakeSubsystem intakeSubsystem) {
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   this.intakeSubsystem = intakeSubsystem;
  //   addRequirements(intakeSubsystem);
    
  // }  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (intakeSubsystem.getBlocked()){
      intakeSubsystem.setCanRun(false);
      counter++; //only invoke if need to have a delay, comment out the line above if you use this
      System.out.println("IS BLOCKED");
    } else {
      intakeSubsystem.setCanRun(true);
      counter = 0;
      System.out.println("IS NOT BLOCKED");
 
    }
    
    if (limelight.getNoteArea() > Constants.noteAreaToRun){
    intakeSubsystem.setIntakeMotor(-.75);

    }
    
    // if (counter > Constants.numSeconds(1.5)) {
    //   intakeSubsystem.setCanRun(false);
    // }   
    
    
    // if (yes){
    //   intakeSubsystem.setIntakeMotor(-.5);
    // } else {
    //   intakeSubsystem.setIntakeMotor(0);
    // }

    // intakeSubsystem.setCanRun(true);
    // if (yes){
    //   intakeSubsystem.setCanRun(true);
    //   counter = 0;
    // }

    // if (intakeSubsystem.getCanRun()){
    //   intakeSubsystem.setIntakeMotor(-.75);

    // } 

    // if (intakeSubsystem.canRun()){
    //   intakeSubsystem.setIntakeMotor(-.75);
      
    // } 
    
    // if (intakeSubsystem.getIntakeVolts() < Constants.intakeNoteVoltage) {
    //   System.out.println("Intake Volts : " + intakeSubsystem.getIntakeVolts());
    //   counter++;
    //   System.out.println("Counter:" + counter);

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
