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
  private double distance = 0;
  /** Creates a new ArmSubsystem. */
  private final TalonFX armMotor = new TalonFX(Constants.armMotor);
  // Initializes an encoder on DIO pins 0 and 1
  // 2X decoding and non-inverted
  Encoder encoder = new Encoder(0, 1, false, Encoder.EncodingType.k1X);

  public ArmSubsystem() {

  }

  public void moveArm(double speed) {
    armMotor.set(speed * .5);
  }

  @Override
  public void periodic() {
    // Gets the distance traveled
    distance = encoder.getDistance();
  }
}
