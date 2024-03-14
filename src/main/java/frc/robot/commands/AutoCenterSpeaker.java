// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class AutoCenterSpeaker extends Command {
// Instance Variables

private double angle;
private final PIDController pidControllerCenter = new PIDController(.04, 0.01, 0); //kp as 0.05 works, everything else as 0 I MADE IT SO MUCH SMOOTHER WTF??? (ty alex Nie)
private final PIDController pidControllerArm = new PIDController(1.4, 0.01, 0);
//dont use I for pid
private LimeLightSubsystem limeLightSubsystem;
private CommandSwerveDrivetrain drivetrain;
private boolean end = false;
private double maxRotationSpeed;
private final SwerveRequest.FieldCentric swerveCentric = new SwerveRequest.FieldCentric();
private int counter = 0;
private int counterForFlywheel = 0;
private Supplier<Double> ySpeed, xSpeed, leftTrigger;
private ArmSubsystem armSubsystem;
private FlywheelSubsystem flywheelSubsystem;
private IntakeSubsystem intakeSubsystem;

  /** Creates a new AutoCenter. */
  public AutoCenterSpeaker(
  Supplier<Double> ySpeed, 
  Supplier<Double> xSpeed, 
  CommandSwerveDrivetrain drivetrain,
  LimeLightSubsystem limeLightSubsystem, 
  double maxRotationSpeed, 
  ArmSubsystem armSubsystem,
  FlywheelSubsystem flywheelSubsystem,
  IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLightSubsystem = limeLightSubsystem;
    this.drivetrain = drivetrain;
    this.maxRotationSpeed = maxRotationSpeed;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.armSubsystem = armSubsystem;
    this.flywheelSubsystem = flywheelSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(limeLightSubsystem);
    addRequirements(drivetrain);
    addRequirements(armSubsystem);
    addRequirements(flywheelSubsystem);
    addRequirements(intakeSubsystem);
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

    angle = limeLightSubsystem.getH_angle(); 
    double speed = pidControllerCenter.calculate(angle,0);
    if (Math.abs(maxRotationSpeed) > 1 ){
      maxRotationSpeed = 1 * Math.signum(maxRotationSpeed);
    }

    if (angle <=1 ){
      end = true;
    }

    // System.out.println("Rotation Speed: " + speed);
    // System.out.println("Angle: " + angle);
    drivetrain.setControl(swerveCentric.withVelocityX(-x).withVelocityY(-y).withRotationalRate(speed));
    
    counter++;
    // distance = limeLightSubsystem.calculateZdistance(Constants.speakerTagHeightMeters);
    // double distance = limeLightSubsystem.getRelative3dBotPose().getZ();
    /* In y = mx + b format (for now, could be a different graph type)
     *  x axis: Distance away from the speaker
     *  y axis: angle of the arm that allowed the note to go in
  
     */
    double distance = limeLightSubsystem.getV_angle();
    //    angle = -.0023 * distance + .3676; original
    double armAngle = -.0023 * distance + .39;
    double armSpeed = pidControllerArm.calculate(armSubsystem.getEncoder(), armAngle);
    // System.out.println("Scoring Speaker PID Speed: " + speed);
    armSubsystem.moveArm(-armSpeed);   


    if (counter > Constants.numSeconds(2)){
      flywheelSubsystem.setFlywheelMotorSpeed();
    }

    if (counter > Constants.numSeconds(2.5)){
      intakeSubsystem.backupIntakeMotor(.75);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    counter = 0;
    flywheelSubsystem.stopFlywheel();
    intakeSubsystem.backupIntakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {//
    // if (end){
    //   end = true;
    //   return true;
    // } else {
    //   return false;
    // }
    return false;
  }
}
