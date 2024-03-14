// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.SystemMenuBar;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class FlyWheelShoot extends Command {
  /** Creates a new AutoFlyWheelShoot. */
  private final FlywheelSubsystem flywheelSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final XboxController xboxController1;
  private final XboxController xboxController2;
  private int counter = 0;
  private int seconds = 0;
  private Supplier<Double> leftTrigger;
  private double isPressed = 0;
  private boolean ampShoot;
  // public FlyWheelShoot(
  //   FlywheelSubsystem flywheelSubsystem,
  //   IntakeSubsystem intakeSubsystem,
  //   XboxController xboxController1,
  //   XboxController xboxController2,
  //   boolean ampShoot) {
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   this.flywheelSubsystem = flywheelSubsystem;
  //   this.intakeSubsystem = intakeSubsystem;
  //   this.xboxController1 = xboxController1;
  //   this.xboxController2 = xboxController2;
  //   this.ampShoot = ampShoot;
  //   addRequirements(flywheelSubsystem);
  //   addRequirements(intakeSubsystem);
  // }

  public FlyWheelShoot(
    FlywheelSubsystem flywheelSubsystem,
    IntakeSubsystem intakeSubsystem,
    XboxController xboxController1,
    XboxController xboxController2,
    Supplier<Double> leftTrigger) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.flywheelSubsystem = flywheelSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.xboxController1 = xboxController1;
    this.xboxController2 = xboxController2;
    this.leftTrigger = leftTrigger;
    addRequirements(flywheelSubsystem);
    addRequirements(intakeSubsystem);
  }  
  // public AutoFlyWheelShoot(FlywheelSubsystem flywheelSubsystem, LimeLightSubsystem limeLight, IntakeSubsystem intakeSubsystem, int seconds) {
  //   // Use addRequirements() here to declare subsystem dependencies.
    
  //   this.flywheelSubsystem = flywheelSubsystem;
  //   this.limeLight = limeLight;
  //   this.intakeSubsystem = intakeSubsystem;
  //   addRequirements(flywheelSubsystem);
  //   addRequirements(limeLight);
  // }  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // flywheelSubsystem.setFlywheelMotorSpeed(.4); //some speed so that it has an easier time getting up to speed
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // in y = mx + b format, need to determine m and b by experiment
    // x axis is distance (limelight z distance)
    // y axis is motorSpeed (for both motors)
    // might include a calculate angle if needed but most likely not needed    
    //we are probably not gonna do this ^ instead just set the motors either to 100% or 50%
    double trigger= leftTrigger.get();
    if (trigger > .1){
    flywheelSubsystem.setFlywheelMotorSpeed();
      counter++;
      if (counter > Constants.numSeconds(.5)){
        intakeSubsystem.backupIntakeMotor(.75);
      }
    } else {
      flywheelSubsystem.stopFlywheel();
      intakeSubsystem.backupIntakeMotor(0);
    }
    
    // double speed = limeLight.calculateZdistance(Constants.speakerTagHeightMeters) * .5 + 1; 

    // flywheelSubsystem.setFlywheelMotorSpeed(speed);
    // counter++;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheelSubsystem.stopFlywheel();
    intakeSubsystem.backupIntakeMotor(0);
    intakeSubsystem.stopRumble(xboxController2);
    counter = 0;

  }

  // Returns true when the command should end.
  @Override
  //TODO: see how long a cycle takes and stop it
  public boolean isFinished() {
    // if (counter > Constants.numSeconds(seconds)){
    //   return true;
    // } else {
    //   return false;
    // }

    return false;
  }
}
