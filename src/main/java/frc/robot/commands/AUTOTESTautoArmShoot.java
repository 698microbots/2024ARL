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

public class AUTOTESTautoArmShoot extends Command {
  /** Creates a new AUTOTESTautoArmShoot. */
  private final ArmSubsystem armSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final LimeLightSubsystem limeLightSubsystem;
  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric swerveRequest = new SwerveRequest.FieldCentric(); //type of field centric is in the class

  private double seconds;
  private final PIDController pidControllerCenter = new PIDController(.04, 0.01, 0); //kp as 0.05 works, everything else as 0 I MADE IT SO MUCH SMOOTHER WTF??? (ty alex Nie)
  private final PIDController pidControllerArm = new PIDController(1.4, 0.01, 0);  
  private int counter;
  public AUTOTESTautoArmShoot(
    ArmSubsystem armSubsystem,
    FlywheelSubsystem flywheelSubsystem,
    IntakeSubsystem intakeSubsystem,
    LimeLightSubsystem limeLightSubsystem,
    CommandSwerveDrivetrain drivetrain,
    double seconds
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.flywheelSubsystem = flywheelSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.limeLightSubsystem = limeLightSubsystem;
    this.drivetrain = drivetrain;
    this.seconds = seconds;
    addRequirements(armSubsystem);
    addRequirements(flywheelSubsystem);
    addRequirements(drivetrain);
    addRequirements(intakeSubsystem);
    addRequirements(limeLightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidControllerCenter.calculate(limeLightSubsystem.getH_angle(),0);
    drivetrain.setControl(swerveRequest.withRotationalRate(speed));
    counter++;
    // double distance = limeLightSubsystem.getV_angle();
    // //    angle = -.0023 * distance + .3676; original
    // double armAngle = -.0023 * distance + .361;
    // double armSpeed = pidControllerArm.calculate(armSubsystem.getEncoder(), armAngle);
    // // System.out.println("Scoring Speaker PID Speed: " + speed);
    double armSpeed = pidControllerArm.calculate(armSubsystem.getEncoder(), Constants.encoderTrap);
    armSubsystem.moveArm(-armSpeed);

    if (counter > Constants.numSeconds(2)){
      flywheelSubsystem.setFlywheelMotorSpeed(1);
    }

    if (counter > Constants.numSeconds(2.5)){
      intakeSubsystem.backupIntakeMotor(.9);
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
  public boolean isFinished() {
    if (counter > Constants.numSeconds(seconds + 3)){
      return true;
    } else {
      return false;
    }
  }
}
