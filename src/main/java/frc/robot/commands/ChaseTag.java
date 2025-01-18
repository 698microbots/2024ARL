// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLightSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChaseTag extends Command {
  /** Creates a new ChaseTag. */
  private double xSpeed = 0;
  private double ySpeed = 0;
  private double omegaSpeed = 0; 

  private LimeLightSubsystem limelight;
  private CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric swerveCentric = new SwerveRequest.FieldCentric(); //might change this to swerve centric
  private final SwerveRequest.RobotCentric roboCentric = new SwerveRequest.RobotCentric(); //might change this to swerve centric

  //trapezoidal motion gives smoother velocity curves (slowing down when getting to target)
  private final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(1, 2);
  private final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(1, 2);
  private final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(2, 3);
  
  private final int TagToChase = 1;
  /*transform 3d will be the pose relative to the target 
   * For this example, Translation3d means that the bot is 1.5m away from the bot in the x direction, with no variance in y or z (directly facing head on)
   * Rotaion3d has no roll or pitch but the camera's rotaiton is 180 degrees flipped (pi in radians) from the target
  */


  //transform so that coordinate system is relative to tag?
  private final Transform3d TagToGoal = new Transform3d(
    new Translation3d(.3, 0, 0), 
    new Rotation3d(0, 0, Math.PI));

  //ProfiledPIDControllers for smoother movement
  private final ProfiledPIDController xController = new ProfiledPIDController(3, .1, 0, xConstraints);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 1, 0, yConstraints);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(1, 20, 0, omegaConstraints);
  
  //periodically updates the pose of the robot for poseEstimator (I'm assuming final can be used because the supplier is final but the values can changing using the .get() method)
  private final Supplier<Pose2d> poseProvider;
  private final Supplier<Pose2d> targetPoseProvider;
  
  
  public ChaseTag(LimeLightSubsystem limelight, CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> poseProvider, Supplier<Pose2d> targetPoseProvider) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.poseProvider = poseProvider;
    this.targetPoseProvider = targetPoseProvider;
    //how close to the target position can you be to say you are at the target position
    xController.setTolerance(.01);
    yController.setTolerance(.01);
    omegaController.setTolerance(Units.degreesToRadians(.1));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //turning the pose2d of the robot into pose3d
    Pose2d robotPose2d = poseProvider.get();
    Pose3d robotPose = new Pose3d(
      robotPose2d.getX(),
      robotPose2d.getY(),
      0,
      new Rotation3d(
        0,
        0,
        robotPose2d.getRotation().getRadians()
      )
    );

    //turning the pose2d of the goal (apriltag) into pose3d
    Pose2d targetPose2d = targetPoseProvider.get();
    Pose3d targetPose = new Pose3d(
      targetPose2d.getX(),
      targetPose2d.getY(),
      0,
      new Rotation3d(
        0,
        0,
        targetPose2d.getRotation().getRadians()
      )
    );

    //transforming the tags pose to the desired goal state pose? not entirely sure
    Pose2d goalPose = targetPose.transformBy(TagToGoal).toPose2d();

    //setting the goal points for the PID
    xController.setGoal(targetPose.getX() + 1);
    yController.setGoal(targetPose.getY());
    omegaController.setGoal(targetPose.getRotation().toRotation2d().getRadians());

    //if there is no tag detected, set all speed to 0
    //else, do the pid calulations and set them to the speeds
    if (limelight.getaprilTagID() == -1){
      
      xSpeed = 0;
      ySpeed = 0;
      omegaSpeed = 0;   
    
    } else {

      xSpeed = xController.calculate(robotPose.getX());
      System.out.println("xSpeed " + xSpeed);
      if (xController.atGoal()){
        drivetrain.setControl(swerveCentric.withVelocityX(0)); 
        System.out.println("x at goal");

      }

      ySpeed = yController.calculate(robotPose.getY());
      System.out.println("ySpeed " + ySpeed);

      if (yController.atGoal()){
        drivetrain.setControl(swerveCentric.withVelocityY(0));
        System.out.println("y at goal");

      }      

      omegaSpeed = omegaController.calculate(robotPose.getRotation().toRotation2d().getRadians());
      System.out.println("omegaSpeed " + omegaSpeed);

      if (omegaController.atGoal()){
        drivetrain.setControl(swerveCentric.withRotationalRate(0));
        System.out.println("omega at goal\n");

      }
      
    }

    //set the speed 
    drivetrain.setControl(swerveCentric.withVelocityX(-0).withVelocityY(0).withRotationalRate(-omegaSpeed));



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(roboCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
