// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final TalonFX armMotor = new TalonFX(Constants.armMotor);
  private final Encoder boreEncoder = new Encoder(0, 1); 
  public ArmSubsystem() {
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  
  public void moveArm(double speed){
    armMotor.set(speed);
  }
  //TODO: find out what this returns
  public double getEncoder(){
    return boreEncoder.get();
  }

  public void resetEncoder(){
    boreEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
