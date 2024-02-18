// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;

public class TESTauto extends Command {
  /** Creates a new AutoTest. */
  private final CommandSwerveDrivetrain driveTrain;
  private int counter;
  private final double seconds;
  private final SwerveRequest.FieldCentric swerveRequest = new SwerveRequest.FieldCentric(); //type of field centric is in the class
  public TESTauto(CommandSwerveDrivetrain driveTrain, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.seconds = seconds;
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // driveTrain.applyRequest(() -> swerveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(.5));
    //applyRequest RETURNS A COMMAND ()
    driveTrain.setControl(swerveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(01));
    
    counter++;
    System.out.println("ITS RUNNING: " + counter);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter > Constants.numSeconds(seconds)){
      return true;
    } else {
      return false;
    }
  }
}
