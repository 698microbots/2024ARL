// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class AutoFlyWheelShoot extends Command {
  /** Creates a new AutoFlyWheelShoot. */
  private final FlywheelSubsystem flywheelSubsystem;
  private final LimeLightSubsystem limeLight;
  private final IntakeSubsystem intakeSubsystem;
  private int counter = 0;
  private int seconds = 0;
  public AutoFlyWheelShoot(FlywheelSubsystem flywheelSubsystem, LimeLightSubsystem limeLight, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.flywheelSubsystem = flywheelSubsystem;
    this.limeLight = limeLight;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(flywheelSubsystem);
    addRequirements(intakeSubsystem);
    addRequirements(limeLight);
  }

  public AutoFlyWheelShoot(FlywheelSubsystem flywheelSubsystem, LimeLightSubsystem limeLight, IntakeSubsystem intakeSubsystem, int seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.flywheelSubsystem = flywheelSubsystem;
    this.limeLight = limeLight;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(flywheelSubsystem);
    addRequirements(limeLight);
  }  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheelSubsystem.setFlywheelMotorSpeed(.4); //some speed so that it has an easier time getting up to speed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // in y = mx + b format, need to determine m and b by experiment
    // x axis is distance (limelight z distance)
    // y axis is motorSpeed (for both motors)
    // might include a calculate angle if needed but most likely not needed    
    double speed = limeLight.calculateZdistance(Constants.speakerTagHeightMeters) * .5 + 1; 

    flywheelSubsystem.setFlywheelMotorSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setCanRun(true);
  }

  // Returns true when the command should end.
  @Override
  //TODO: see how long a cycle takes and stop it
  public boolean isFinished() {
    return false;
  }
}
