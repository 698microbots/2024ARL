// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class AUTOTESTIntakeMoveAndDriveTrain extends Command {
  /** Creates a new TESTIntakeMoveAndDriveTrain. */
  private final IntakeSubsystem intakeSubsystem;
  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric swerveRequest = new SwerveRequest.FieldCentric(); //type of field centric is in the class
  private double x,y,theta;
  private double seconds;
  private int counter = 0;
  public AUTOTESTIntakeMoveAndDriveTrain(
    IntakeSubsystem intakeSubsystem, 
    CommandSwerveDrivetrain drivetrain, 
    double seconds,
    double x,
    double y,
    double theta) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.drivetrain = drivetrain;
    this.seconds = seconds;
    this.x = x;
    this.y = y;
    this.theta = theta;
    addRequirements(intakeSubsystem);
    addRequirements(drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    intakeSubsystem.setIntakeMotor(.25);
    drivetrain.setControl(swerveRequest.withVelocityX(x).withVelocityY(y).withRotationalRate(theta));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    counter = 0;
    intakeSubsystem.backupIntakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter > Constants.numSeconds(seconds)){
      return true;
    } else {
      return false;
    }
  }
}
