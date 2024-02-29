// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.*;

public class LimeLightSubsystem extends SubsystemBase {
  // instance variables
  private Translation2d translation2d = new Translation2d();

  // creates the instance variables for the LimeLight Subsystem
  private NetworkTable limeLight, limeLight2;
  private NetworkTableEntry V_angle, H_angle, hasTargets, botPose, aprilID;
  private double[] poseList;
  private double zDistance;
  private double xDistance;

  /** Creates a new LimeLight. */
  public LimeLightSubsystem() {
    limeLight = NetworkTableInstance.getDefault().getTable("limelight");
    limeLight2 = NetworkTableInstance.getDefault().getTable("limelighttwo");
    //make sure all the keys are exactly matching the docs, no capitals 
    V_angle = limeLight.getEntry("ty");
    H_angle = limeLight.getEntry("tx");
    hasTargets = limeLight.getEntry("tv");
    botPose = limeLight.getEntry("targetpose_robotspace");
    aprilID = limeLight.getEntry("tid");
  }

  // getters
  public double hasTargets() {
    return hasTargets.getDouble(0);
  }

  public double getV_angle() {
    return V_angle.getDouble(0);
  }

  public double getH_angle() {
    return H_angle.getDouble(0);
  }

  public Pose2d get2dBotPoseForAmp() {
    /*
     * Its specific because it determines what type of botpose we need
     * For example, we may need the botpose, botpose_wpiblue, botpose_wpired, etc
     * in order to tell our distance from the apriltag.
     * This method should give us an x and y position to the april tag as well as a rotaiton angle to it
     */
    poseList = botPose.getDoubleArray(new double[6]);
    //position
    double x = poseList[0];
    double y = poseList[1];
    double z = poseList[2];
    //rotation
    double roll = poseList[3];
    double pitch = poseList[4];
    double yaw = poseList[5];

    Pose3d pose3d = new Pose3d(
    x,
    y,
    z,
    new Rotation3d(
      roll,
      pitch,
      yaw
    ));
    return pose3d.toPose2d();
  }

  public Pose3d get3dBotPoseForAmp() {
    /*
     * Its specific because it determines what type of botpose we need
     * For example, we may need the botpose, botpose_wpiblue, botpose_wpired, etc
     * in order to tell our distance from the apriltag.
     * This method should give us an x and y position to the april tag as well as a rotaiton angle to it
     */
    poseList = botPose.getDoubleArray(new double[6]);
    //position
    double x = poseList[0];
    double y = poseList[1];
    double z = poseList[2];
    //rotation
    double roll = poseList[3];
    double pitch = poseList[4];
    double yaw = poseList[5];

    Pose3d pose3d = new Pose3d(
    x,
    y,
    z,
    new Rotation3d(
      roll,
      pitch,
      yaw
    ));
    return pose3d;
  }  
  public double getaprilTagID() {
    return aprilID.getDouble(0);

  }

  public double getTargetPoseX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
        .getDoubleArray(new double[6])[0];
  }

  public double getTargetPoseZ() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
        .getDoubleArray(new double[6])[1];
  }

  public double getRobotPoseX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace")
        .getDoubleArray(new double[6])[0];
  }

  public double getRobotPoseZ() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace")
        .getDoubleArray(new double[6])[3];
  }

  // setters
  public void setPipeline(int pipe) {
    limeLight.getEntry("pipeline").setNumber(pipe);
  }
  // 0: AprilTag
  // 1: Reflective
  // 2: Zoomed In

  public double calculateZdistance(double goalHeight) {// Z direction is foward from the robot
    zDistance = (goalHeight - Constants.limeLightHeight)
        * (Math.tan(Math.toRadians(getV_angle() + Constants.limeLightInitAngle)));

    return zDistance;

  }

  // public double calculateXdistance() {// X direction is sideways from the robot
  //   xDistance = calculateZdistance() * Math.tan(Math.toRadians(getH_angle()));
  //   return xDistance;
  // }

  public double getXDist() {
    // return Math.abs(getRobotPoseX() - getTargetPoseX());
    return translation2d.getX();
  }

  public double getYDist() {
    // return Math.abs(getRobotPoseY() - getRobotPoseY());
    return translation2d.getY();
  }


  @Override
  public void periodic() {
    /// This method will be called once per scheduler run
    /// calculateXdistance();
    /// calculateZdistance();
    // the methods to get x and y distances go here
  }

  // public void setLight(boolean on) {
  //   limeLight.getEntry("ledMode").setNumber(1);
  // }
}
