// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.constant.Constable;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

public class AutoPosition extends Command {
  // instance variables
  private double hypot = 0;
  private double turnAngle = 0;
  private double yDisp = 0;
  private double xDisp = 0;
  // private boolean blueAlliance; //if you are on the blue alliance or not (for robot pose)
  //made these visible to the entire class so isFinished() can see it
  private double currentX;
  private double currentY;
  private double currentAngle;

  
  
  
  private LimeLightSubsystem limeLightSubsystem;
  private final SwerveRequest.RobotCentric swerveCentric = new SwerveRequest.RobotCentric();
  private CommandSwerveDrivetrain drivetrain;
  private PIDController pidControllerXController = new PIDController(.3, 0, 0); // TODO - tune this
  private PIDController pidControllerYController = new PIDController(.3, 0, 0); // TODO - tune this
  private PIDController pidControllerAngleController = new PIDController(.3, 0, 0); // TODO - tune this

  /** Creates a new AutoPosition. */
  public AutoPosition(CommandSwerveDrivetrain drivetrain, LimeLightSubsystem limeLightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLightSubsystem = limeLightSubsystem;
    this.drivetrain = drivetrain;
    addRequirements(limeLightSubsystem);
    addRequirements(drivetrain);
    // turnAngle = limeLightSubsystem.getH_angle();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xDisp = limeLightSubsystem.get2dBotPoseForAmp().getX(); //just gets the initial distances
    yDisp = limeLightSubsystem.get2dBotPoseForAmp().getY();
    // turnAngle = Units.radiansToDegrees(limeLightSubsystem.get2dBotPoseForAmp().getRotation().getRadians());
    // angle constantly changes so this does not need to be here
  }
  //TODO: for pose, it initializes from the blue alliance driver station, get target pose and robot pose to estimate distance from the tags
  //maybe get poses from specific alliances
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //this is like kind of complicated so it probalby wont work but we will see
    hypot = Math.hypot(xDisp, yDisp);

    currentX = limeLightSubsystem.get2dBotPoseForAmp().getX(); //just gets the initial distances
    currentY = limeLightSubsystem.get2dBotPoseForAmp().getY();
    currentAngle = Units.radiansToDegrees(limeLightSubsystem.get2dBotPoseForAmp().getRotation().getRadians());
  
    double xSpeed = pidControllerXController.calculate(currentX, 0);
    double ySpeed = pidControllerYController.calculate(currentY, 0);
    double rotateSpeed = pidControllerAngleController.calculate(currentAngle, 0);
    //we might have to change x and y speed depending on what the bot actually thinks is x and y direction
    
    System.out.println("x speed: " + xSpeed);
    System.out.println("y speed: " + ySpeed);
    System.out.println("rotate speed: " + rotateSpeed);

    // TODO: uncomment this once speeds are reasonable
    // drivetrain.setControl(swerveCentric.
    //   withVelocityX(xSpeed).
    //   withVelocityY(ySpeed).
    //   withRotationalRate(rotateSpeed));


    // drivetrain.setControl(swerveCentric.withRotationalRate(pidController.calculate(limeLightSubsystem.getBotPose(), limeLightSubsystem.getH_angle())));
    // drivetrain.setControl(swerveCentric.withVelocityX(
    //     pidController.calculate(limeLightSubsystem.getRobotPoseX(), limeLightSubsystem.getTargetPoseX())));
    // drivetrain.setControl(swerveCentric.withVelocityY(
    //     pidController.calculate(limeLightSubsystem.getRobotPoseZ(), limeLightSubsystem.getTargetPoseZ())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentX < 1 || currentY < 1 || currentAngle < 1){
      return true;
    } else {
      return false;
    }
  }
}
