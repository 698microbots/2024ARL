// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class AutoCenterNoteAndIntake extends Command {
  /** Creates a new AutoCenterNoteAndIntake. */
private double angle;
private PIDController pidController = new PIDController(.04, 0.01, 0.00); //kp as 0.05 works, everything else as 0
//dont use I for pid
private LimeLightSubsystem limeLightSubsystem;
private CommandSwerveDrivetrain drivetrain;
private final SwerveRequest.FieldCentric swerveCentric = new SwerveRequest.FieldCentric(); //might change this to swerve centric
private double maxRotationSpeed;
private Supplier<Double> ySpeed, xSpeed;
private IntakeSubsystem intakeSubsystem; 
private LightSubsystem lightSubsystem;
private XboxController xbox1, xbox2;
  public AutoCenterNoteAndIntake(
      Supplier<Double> ySpeed,
      Supplier<Double> xSpeed,
      CommandSwerveDrivetrain drivetrain,
      LimeLightSubsystem limeLightSubsystem,
      double maxRotationSpeed,
      IntakeSubsystem intakeSubsystem,
      LightSubsystem lightSubsystem,
      XboxController xbox1,
      XboxController xbox2) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ySpeed = ySpeed;
    this.xSpeed = xSpeed;
    this.drivetrain = drivetrain;
    this.limeLightSubsystem = limeLightSubsystem;
    this.maxRotationSpeed = maxRotationSpeed;
    this.intakeSubsystem = intakeSubsystem;
    this.lightSubsystem = lightSubsystem;
    this.xbox1 = xbox1;
    this.xbox2 = xbox2;
    addRequirements(lightSubsystem);
    addRequirements(drivetrain);
    addRequirements(intakeSubsystem);
    addRequirements(limeLightSubsystem);
  }

  public AutoCenterNoteAndIntake(Supplier<Double> xSpeed, Supplier<Double> ySpeed, CommandSwerveDrivetrain drivetrain, LimeLightSubsystem limeLightSubsystem,
      double maxRotationSpeed, IntakeSubsystem intakeSubsystem, LightSubsystem lightSubsystem, XboxController xbox2,
      XboxController xbox1, int numSeconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ySpeed = ySpeed;
    this.xSpeed = xSpeed;
    this.drivetrain = drivetrain;
    this.limeLightSubsystem = limeLightSubsystem;
    this.maxRotationSpeed = maxRotationSpeed;
    this.intakeSubsystem = intakeSubsystem;
    this.lightSubsystem = lightSubsystem;
    this.xbox1 = xbox1;
    this.xbox2 = xbox2;
    this.numSeconds = numSeconds;
    addRequirements(lightSubsystem);
    addRequirements(drivetrain);
    addRequirements(intakeSubsystem);
    addRequirements(limeLightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = xSpeed.get();
    double y = ySpeed.get();
    System.out.println(intakeSubsystem.getBlocked());

    if (intakeSubsystem.getBlocked()) {
      intakeSubsystem.setCanRun(false);
      lightSubsystem.setLights(Constants.colorRGBIntake[0], Constants.colorRGBIntake[1], Constants.colorRGBIntake[2],
          .5);
      intakeSubsystem.rumbleController(xbox1, 1);
      intakeSubsystem.rumbleController(xbox2, 1);
      System.out.println("IS BLOCKED");
    } else {
      intakeSubsystem.setCanRun(true);
      System.out.println("IS NOT BLOCKED");
      lightSubsystem.setLights(0, 0, 0, .5);

    }

    if (limeLightSubsystem.getNoteArea() > Constants.noteAreaToRun) {
      intakeSubsystem.setIntakeMotor(.75);
    }
    intakeSubsystem.rumbleController(xbox1);
    intakeSubsystem.rumbleController(xbox2);
    angle = limeLightSubsystem.getNoteHorizontalAngle();
    double rotationSpeed = pidController.calculate(angle, 0);
    if (Math.abs(maxRotationSpeed) > 1) {
      } 
    intakeSubsystem.rumbleController(xbox2);   
    angle = limeLightSubsystem.getNoteHorizontalAngle(); 
    if (Math.abs(maxRotationSpeed) > 1 ){
      maxRotationSpeed = 1 * Math.signum(maxRotationSpeed);
    }

    // System.out.println("Rotation Speed: " + rotationSpeed);
    // System.out.println("Angle: " + angle);
    drivetrain.setControl(swerveCentric.withVelocityX(-x).withVelocityY(-y).withRotationalRate(rotationSpeed));
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeMotor(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter < Constants.numSeconds(numSeconds)) {
      return true;
    } else {}
  return false;
  }
}
