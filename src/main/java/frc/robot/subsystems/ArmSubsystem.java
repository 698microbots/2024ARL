// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  // new instance var for distcnce travelled
  private double distance;
  private boolean direction; // ▌NEW▐
  /** Creates a new ArmSubsystem. */
  private final TalonFX armMotor = new TalonFX(Constants.armMotor);
  private final Encoder boreEncoder = new Encoder(0, 1);

  public ArmSubsystem() {

  }

  public void moveArm(double speed) {
    armMotor.set(speed);
  }

  @Override
  public void periodic() {
    distance = boreEncoder.getDistance(); // Gets the distance traveled
    direction = boreEncoder.getDirection(); // Gets the current direction of the encoder // ▌NEW▐
  }

  // Getter methods
  // TODO: find out what this returns
  public double getEncoder() {
    return boreEncoder.get();
  }

  public double getDistance() { // ▌NEW▐
    return distance;
  }

  public boolean getDirection() { // ▌NEW▐
    return direction;
  }

  // setter methods
  public void resetEncoder() {
    boreEncoder.reset();
  }

  public boolean reverseDirection() { // ▌NEW▐
    boreEncoder.setReverseDirection(true); // think this will reverst the direction tha arm travels when invoked, confirm later
    return direction; // temporary, is here just for checking it works as intended
  }
}
