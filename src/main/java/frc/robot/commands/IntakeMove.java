// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class IntakeMove extends Command { // TODO - add CANdle (led strips) functionality
  /** Creates a new IntakeMove. */
  private final IntakeSubsystem intakeSubsystem;
  private CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric swerveCentric = new SwerveRequest.FieldCentric();

  private final LimeLightSubsystem limelight;
  private final LightSubsystem lightSubsystem;
  // private  boolean yes = false;
  private int counter = 0;
  private boolean reverse;
  private XboxController xboxController1;
    private XboxController xboxController2;
  private Supplier<Double> x, y, theta;
  
  public IntakeMove(
    XboxController xboxController1, 
    XboxController xboxController2, 
    IntakeSubsystem intakeSubsystem, 
    LimeLightSubsystem limelight, 
    boolean reverse,
    LightSubsystem lightSubsystem
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.limelight = limelight;
    this.reverse = reverse;
    this.xboxController1 = xboxController1;
    this.xboxController2 = xboxController2;
    this.lightSubsystem = lightSubsystem;

    // this.yes = yes;
    addRequirements(intakeSubsystem);
    addRequirements(limelight);
    
  }  
  
  
  public IntakeMove(
    XboxController xboxController1, 
    XboxController xboxController2, 
    IntakeSubsystem intakeSubsystem, 
    LimeLightSubsystem limelight, 
    boolean reverse,
    LightSubsystem lightSubsystem,
    CommandSwerveDrivetrain drivetrain,
    Supplier<Double> x,
    Supplier<Double> y,
    Supplier<Double> theta) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.reverse = reverse;
    this.xboxController1 = xboxController1;
    this.xboxController2 = xboxController2;
    this.lightSubsystem = lightSubsystem;
    this.x = x;
    this.y = y;
    this.theta = theta;
    // this.yes = yes;
    addRequirements(intakeSubsystem);
    addRequirements(drivetrain);
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
  
  if (!reverse){
    intakeSubsystem.setIntakeMotor(.5);
    drivetrain.setControl(swerveCentric.withVelocityX(-y.get()).withVelocityY(-x.get()).withRotationalRate(-theta.get() * Math.PI));

    // if (intakeSubsystem.getBlocked()){
    //   intakeSubsystem.setCanRun(false);
    //   lightSubsystem.setLights(Constants.colorRGBIntake[0], Constants.colorRGBIntake[1], Constants.colorRGBIntake[2]);
    //   counter++; //only invoke if need to have a delay, comment out the line above if you use this
    //   // System.out.println("IS BLOCKED");
    //   intakeSubsystem.rumbleController(xboxController1);
    //   intakeSubsystem.rumbleController(xboxController2);
    // } else {
    //   intakeSubsystem.setCanRun(true);
    //   lightSubsystem.setLights(0, 0 ,0);
    //   intakeSubsystem.rumbleController(xboxController1);
    //   intakeSubsystem.rumbleController(xboxController2);      
    //   counter = 0;
    //   // System.out.println("IS NOT BLOCKED");
    // }
    
    // if (limelight.getNoteArea() > Constants.noteAreaToRun && intakeSubsystem.getCanRun()){
    // intakeSubsystem.setIntakeMotor(.75);
    // }
  
  } else {
    intakeSubsystem.reverseIntakeMotor(.5);
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
