// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightSubsystem;

public class AutoPosition extends Command {
  // instance variables
  private double hypot = 0;
  private double turnAngle = 0;
  private double orientationAngle = 90 - turnAngle;
  private double zDisp = 0; 
  private double xDisp = 0;
  private LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();

  /** Creates a new AutoPosition. */
  public AutoPosition(LimeLightSubsystem limeLightSubsystem) {
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
    zDisp = limeLightSubsystem.calculateZdistance();
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
