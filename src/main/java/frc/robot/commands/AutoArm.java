// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class AutoArm extends Command {
  /** Creates a new AutoArm. */
  private double encoderPosition;
  private ArmSubsystem armSubsystem;
  private LimeLightSubsystem limeLightSubsystem;
  private PIDController pidController = new PIDController(0.001,
   0, 0);
  private boolean scoringAmp;
  private double speed = 0;
  private double distance = 0;
  private double angle = 0;
  public AutoArm(ArmSubsystem armSubsystem, boolean scoringAmp, LimeLightSubsystem limeLightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.scoringAmp = scoringAmp;
    this.limeLightSubsystem = limeLightSubsystem;
    addRequirements(armSubsystem);
    addRequirements(limeLightSubsystem);
  }
  //TODO: before testing anything arm related, test the encoders first
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (scoringAmp){
      speed = pidController.calculate(armSubsystem.getEncoder(), Constants.ampEncoderUnits);
      System.out.println("Scoring Amp PID Speed: " + speed);
      // armSubsystem.moveArm(speed);
    } else {
      // distance = limeLightSubsystem.calculateZdistance(Constants.speakerTagHeightMeters);
      distance = limeLightSubsystem.getRelative3dBotPose().getZ();
      /* In y = mx + b format (for now, could be a different graph type)
       *  x axis: Distance away from the speaker
       *  y axis: angle of the arm that allowed the note to go in
       * 
       */
      angle = 0.5 * distance + 1;
      speed = pidController.calculate(armSubsystem.getEncoder(), angle);
      System.out.println("Scoring Speaker PID Speed: " + speed);
    }







  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.moveArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  // if the speed from the pidController is less than a certain number
    if (Math.abs(speed) < Constants.endArmPositionSpeed){
      return true;
    } else {
      return false;
    }
  
  }
}
