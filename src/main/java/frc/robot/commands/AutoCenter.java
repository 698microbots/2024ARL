// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimeLightSubsystem;

public class AutoCenter extends Command {
// Instance Variables
private double hypot = 0;
private double turnAngle = 0;
private double orientationAngle = 90 - turnAngle;
private double xDisp = 0;
private double zDisp = 0; 
private LimeLightSubsystem limeLightSubsystem;

  /** Creates a new AutoCenter. */
  public AutoCenter(LimeLightSubsystem limeLightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLightSubsystem = limeLightSubsystem;
    addRequirements(limeLightSubsystem);
    hypot = limeLightSubsystem.calcHypotenuse();
    turnAngle = limeLightSubsystem.getH_angle();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimeLightSubsystem limeLight = new LimeLightSubsystem(); // new limelight object
    xDisp = limeLight.calculateXdistance();
    zDisp = limeLight.calculateZdistance();
    
    // tell the motors to move & turn

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(xDisp == 0 && zDisp == 0) {
      return true;
    } else {
      return false;
    }
  }
}
