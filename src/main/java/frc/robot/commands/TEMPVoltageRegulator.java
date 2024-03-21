// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.driveTrainVoltages;

public class TEMPVoltageRegulator extends Command {
  private IntakeSubsystem intakeSubsystem;
  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  private ArmSubsystem armSubsystem;
  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

  private Supplier<Double> xSpeed;
  private Supplier<Double> ySpeed;
  private Supplier<Double> rotationSpeed; // multiple by 1.5 * pi

  /** Creates a new TMPVoltageRegulator. */
  public TEMPVoltageRegulator(IntakeSubsystem intakeSubsystem, CommandSwerveDrivetrain commandSwerveDrivetrain, ArmSubsystem armSubsystem, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotationSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.armSubsystem = armSubsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotationSpeed = rotationSpeed;
    addRequirements(armSubsystem);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    commandSwerveDrivetrain.setControl(fieldCentric.withVelocityX(- xSpeed.get()).withVelocityY(- ySpeed.get()).withRotationalRate(rotationSpeed.get() * (1.5 * Math.PI)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    commandSwerveDrivetrain.setControl(fieldCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
