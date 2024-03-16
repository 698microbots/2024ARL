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
  private final PIDController pidControllerArm = new PIDController(1.4, 0.01, 0);
  private int counter = 0;
  public AutoScoreSpeakerArm(
    ArmSubsystem armSubsystem
    ) {
    // Use addRequirements() here to declare subsystem dependencies.
  this.armSubsystem = armSubsystem;
  addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armSpeed = pidControllerArm.calculate(armSubsystem.getEncoder(), Constants.encoderTrap);
    armSubsystem.moveArm(-armSpeed);
    System.out.println("testing");
    // if (counter > Constants.numSeconds(2)){
    //   flywheelSubsystem.setFlywheelMotorSpeed();
    // }

    // if (counter > Constants.numSeconds(2.5)){
    //   intakeSubsystem.backupIntakeMotor(.75);
    // } 
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.moveArm(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
