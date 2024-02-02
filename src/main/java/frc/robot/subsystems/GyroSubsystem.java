// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {
  /** Creates a new GyroSubsystem. */
  private final Pigeon2 gyro = new Pigeon2(0);

  public GyroSubsystem() {

  }

  public double getAngle(){
    return gyro.getAngle() % 360;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
