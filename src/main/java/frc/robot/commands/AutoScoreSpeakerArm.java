// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoScoreSpeakerArm extends Command {
  /** Creates a new AutoScoreTrap. */
  private final ArmSubsystem armSubsystem;
  private final PIDController pidControllerArm = new PIDController(1, 0.0, 0);
  private final FlywheelSubsystem flywheelSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private int counter = 0;
  public AutoScoreSpeakerArm(
    ArmSubsystem armSubsystem,
    FlywheelSubsystem flywheelSubsystem,
    IntakeSubsystem intakeSubsystem
    ) {
    // Use addRequirements() here to declare subsystem dependencies.
  this.armSubsystem = armSubsystem;
  this.intakeSubsystem = intakeSubsystem;
  this.flywheelSubsystem = flywheelSubsystem;
  addRequirements(armSubsystem);
  addRequirements(intakeSubsystem);
  addRequirements(flywheelSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armSpeed = pidControllerArm.calculate(armSubsystem.getEncoder(), Constants.encoderManualSpeaker);
    counter++;
    armSubsystem.moveArm(-armSpeed);
    // System.out.println("testing");
    if (counter > Constants.numSeconds(1.2)){
      flywheelSubsystem.setFlywheelMotorSpeed(1);
    }

    if (counter > Constants.numSeconds(1.7)){
      intakeSubsystem.backupIntakeMotor(.75);
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
