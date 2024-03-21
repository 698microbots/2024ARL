// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  // new instance var for distcnce travelled
  private double distance = 0;
  private boolean direction = false;
  private boolean runArm;
  /** Creates a new ArmSubsystem. */
  private final TalonFX armMotor = new TalonFX(Constants.armMotor);
  private final TalonFX armMotor2 = new TalonFX(Constants.armMotor2);
  // Initializes a duty cycle encoder on DIO pins 0
  DutyCycleEncoder dutyCycleEncoder = new DutyCycleEncoder(0);

  public ArmSubsystem() {
    // armMotor.setNeutralMode(NeutralModeValue.Brake);
    // armMotor2.setNeutralMode(NeutralModeValue.Brake);
    armMotor.setNeutralMode(NeutralModeValue.Coast);
    armMotor.setNeutralMode(NeutralModeValue.Coast);


  }

  public void moveArm(double speed) {
    armMotor.set(speed);
    armMotor2.set(-speed);
    if (speed > 0) {
      runArm = true;
    } else {
      runArm = false;
    }
  }

  // @Override
  // public void periodic() { // don't need preiodic
  // distance = ; // Gets the distance traveled
  // direction = ; // Gets the current direction of the encoder
  // }

  // Getter methods
  // TODO: find out what this returns
  public double getEncoder() {
    return dutyCycleEncoder.get();
  }

  public double getDistance() {
    return dutyCycleEncoder.getDistance();
  }

  // setter methods
  public void resetEncoder() {
    dutyCycleEncoder.reset();
  }

  public boolean getRunArm() {
    return runArm;
  }

  // public boolean reverseDirection() {
  //   dutyCycleEncoder.setReverseDirection(true); // think this will reverst the direction tha arm travels when invoked,
  //                                          // confirm later
  //   return direction; // temporary, is here just for checking it works as intended
  // }
}
