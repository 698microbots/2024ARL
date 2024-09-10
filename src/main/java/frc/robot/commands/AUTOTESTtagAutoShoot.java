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

public class AUTOTESTtagAutoShoot extends Command {
  /** Creates a new AUTOTESTtagAutoShoot. */
private final PIDController pidControllerCenter = new PIDController(.04, 0.01, 0); //kp as 0.05 works, everything else as 0 I MADE IT SO MUCH SMOOTHER WTF??? (ty alex Nie)
private final PIDController pidControllerArm = new PIDController(1, 0, 0); //working arm constants  
private int counter = 0;
private final LimeLightSubsystem limeLightSubsystem;
private final IntakeSubsystem intakeSubsystem;
private final ArmSubsystem armSubsystem;
private final SwerveRequest.FieldCentric swerveCentric = new SwerveRequest.FieldCentric();
private final CommandSwerveDrivetrain drivetrain;
private final FlywheelSubsystem flywheelSubsystem;
public AUTOTESTtagAutoShoot(LimeLightSubsystem limeLightSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, CommandSwerveDrivetrain drivetrain, FlywheelSubsystem flywheelSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLightSubsystem = limeLightSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.armSubsystem = armSubsystem;
    this.drivetrain = drivetrain;
    this.flywheelSubsystem = flywheelSubsystem;
    addRequirements(limeLightSubsystem);
    addRequirements(intakeSubsystem);
    addRequirements(armSubsystem);
    addRequirements(drivetrain);
    addRequirements(flywheelSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidControllerCenter.calculate(limeLightSubsystem.getH_angle(),0);
    


    // System.out.println("Rotation Speed: " + speed);
    // System.out.println("Angle: " + angle);
    drivetrain.setControl(swerveCentric.withRotationalRate(speed));
    
    counter++;  
    
    
    double distance = limeLightSubsystem.getV_angle();
    //    angle = -.0023 * distance + .3676; original
    double armAngle = -.0023 * distance + .39;
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

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter > Constants.numSeconds(2.6)){
      return true;
    } else {
      return false;
    }
  }
}
