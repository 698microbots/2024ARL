// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class driveTrainVoltages extends SubsystemBase {
  /** Creates a new driveTrainVoltages. */
  private final TalonFX FLDrive = new TalonFX(Constants.frontLeftDrive);
  private final TalonFX BLDrive = new TalonFX(Constants.backLeftDrive);
  private final TalonFX FRDrive = new TalonFX(Constants.frontRightDrive);
  private final TalonFX BRDrive = new TalonFX(Constants.backRightDrive);
 
  private final TalonFX FLTurn = new TalonFX(Constants.frontLeftTurn);
  private final TalonFX BLTurn = new TalonFX(Constants.backLeftTurn);
  private final TalonFX BRTurn = new TalonFX(Constants.backRightTurn);
  private final TalonFX FRTurn = new TalonFX(Constants.frontRightTurn);

  
  public driveTrainVoltages() {}

  public double FLDVoltage(){
    return FLDrive.getMotorVoltage().getValueAsDouble();
  }

  public double BLDVoltage(){
    return BLDrive.getMotorVoltage().getValueAsDouble();
  }
  
  public double FRDVoltage(){
    return FRDrive.getMotorVoltage().getValueAsDouble();
  }
  
  public double BRDVoltage(){
    return BRDrive.getMotorVoltage().getValueAsDouble();
  }

  //

  public double FLTVoltage(){
    return FLTurn.getMotorVoltage().getValueAsDouble();
  }

  public double BLTVoltage(){
    return BLTurn.getMotorVoltage().getValueAsDouble();
  }
  
  public double FRTVoltage(){
    return FRTurn.getMotorVoltage().getValueAsDouble();
  }
  
  public double BRTVoltage(){
    return BRTurn.getMotorVoltage().getValueAsDouble();
  }


  public double FLTVelocity(){
    return FLDrive.getVelocity().getValueAsDouble();
  }

  public double BLVelocity(){
    return BLDrive.getVelocity().getValueAsDouble();
  }
  
  public double FRVelocity(){
    return FRDrive.getVelocity().getValueAsDouble();
  }
  
  public double BRVelocity(){
    return BRDrive.getVelocity().getValueAsDouble();
  }  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
