// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class AutoTrapFromGround extends Command {
  /** Creates a new AutoTrapFromGround. */
  private final IntakeSubsystem intakeSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;
  private final LimeLightSubsystem limeLightSubsystem;
  private final CommandSwerveDrivetrain drivetrain;
  private final ArmSubsystem armSubsystem;
  private final PIDController pidControllerArm = new PIDController(1, 0, 0); //working arm constants
  private int counter = 0;
  private final SwerveRequest.FieldCentric swerveCentric = new SwerveRequest.FieldCentric();

  private final PIDController pidControllerCenter = new PIDController(.04, 0.01, 0); //kp as 0.05 works, everything else as 0 I MADE IT SO MUCH SMOOTHER WTF??? (ty alex Nie)

  public AutoTrapFromGround(IntakeSubsystem intakeSubsystem, FlywheelSubsystem flywheelSubsystem, LimeLightSubsystem limeLightSubsystem, CommandSwerveDrivetrain drivetrain, ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.flywheelSubsystem = flywheelSubsystem;
    this.limeLightSubsystem = limeLightSubsystem;
    this.drivetrain = drivetrain;
    this.armSubsystem = armSubsystem;
    addRequirements(intakeSubsystem);
    addRequirements(flywheelSubsystem);
    addRequirements(limeLightSubsystem);
    addRequirements(drivetrain);
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    double speed = pidControllerCenter.calculate(limeLightSubsystem.getH_angle(),-7);
    drivetrain.setControl(swerveCentric.withRotationalRate(speed));

    double distance = limeLightSubsystem.getV_angle();
    //    angle = -.0023 * distance + .3676; original
    // double armAngle = 0.0005 * distance + .3643;
    double armAngle = 0.0005 * distance + .397;

    double armSpeed = pidControllerArm.calculate(armSubsystem.getEncoder(), armAngle);
    // System.out.println("Scoring Speaker PID Speed: " + speed);
    armSubsystem.moveArm(-armSpeed);     
    if (counter > Constants.numSeconds(2)){
      flywheelSubsystem.setFlywheelMotorSpeed(1);
    }

    if (counter > Constants.numSeconds(2.5)){
      intakeSubsystem.backupIntakeMotor(.75);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheelSubsystem.setFlywheelMotorSpeed(0);
    intakeSubsystem.backupIntakeMotor(0);
    armSubsystem.moveArm(0);
    counter = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
