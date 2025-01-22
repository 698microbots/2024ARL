// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLightSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChaseTagNew extends Command {
  /** Creates a new ChaseTagNew. */

  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();


  private final PIDController pidControllerX = new PIDController(1, 0.1, 0);
  private final PIDController pidControllerY = new PIDController(1, 0.1, 0);
  private final PIDController pidControllerOmega = new PIDController(.05, .01, 0);

  private LimeLightSubsystem limeLightSubsystem;
  private CommandSwerveDrivetrain drivetrain;

  public ChaseTagNew(LimeLightSubsystem limeLightSubsystem, CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLightSubsystem = limeLightSubsystem;
    this.drivetrain = drivetrain;
    addRequirements(limeLightSubsystem);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double omegaSpeed = pidControllerOmega.calculate(limeLightSubsystem.getH_angle(), 0);
    // System.out.println("omegaSpeed " + omegaSpeed);


    if (limeLightSubsystem.getaprilTagID() == -1){

      drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));

    } else {
      double xSpeed = pidControllerX.calculate(limeLightSubsystem.getRelative3dBotPose().getZ(), -1.3);
      // System.out.println("xSpeed " + xSpeed);
      System.out.println("y pos " + limeLightSubsystem.getRelative3dBotPose().getY());

    double ySpeed = pidControllerY.calculate(limeLightSubsystem.getRelative3dBotPose().getX(), 0);
    // System.out.println("ySpeed " + ySpeed);

    // drivetrain.setControl(swerveCentric.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(omegaSpeed));

    drivetrain.setControl(robotCentric.withVelocityX(-xSpeed).withVelocityY(ySpeed).withRotationalRate(omegaSpeed));
 

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
