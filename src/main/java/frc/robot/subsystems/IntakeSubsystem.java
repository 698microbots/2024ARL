// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  // private final CANSparkMax IntakeMotor = new CANSparkMax(15, CANSparkMax.MotorType.kBrushless);
  private final TalonFX IntakeMotorNew = new TalonFX(Constants.intakeMotorNew);
  // Initializes a DigitalInput on DIO 0
  private final DigitalInput photoSensor = new DigitalInput(1); //TODO - make this a constant
  private final DigitalInput photoSensor2 = new DigitalInput(3); //TODO - make this a constant
  
  private boolean canRun = true;


  public IntakeSubsystem() {

  }

  public void setIntakeMotor(double speed) {
    if (getBlocked()) {
      // IntakeMotor.set(0);
      // System.out.println("stopping motor");
      IntakeMotorNew.set(0);
    } else {
      // IntakeMotor.set(-speed);
      // System.out.println("setting motor");
      IntakeMotorNew.set(speed); // dont know whether we need -speed or positive
    
    }
  }

  //also use to override normal setIntakeMotor
  public void reverseIntakeMotor(double speed) {
    // IntakeMotor.set(speed);
    IntakeMotorNew.set(-speed);
  }

  public void backupIntakeMotor(double speed){
    // IntakeMotor.set(-speed);
    IntakeMotorNew.set(speed);
  }
  // public double getIntakeVolts() {
  //   return IntakeMotor.getBusVoltage();
  // }

  public void setCanRun(boolean run) {
    // if (run){
    //   System.out.println("I can run");
    // } else {
    //   System.out.println("I CANT  run");
    // }
    canRun = run;
  }

  public boolean getCanRun() {
    return canRun;
  }

  // public boolean canRun() {
  //   if (getBlocked()) {
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }

  public boolean getBlocked() {
    return !photoSensor.get();
  }

  public boolean getBlocked2(){
    return !photoSensor2.get();
  }

  public void rumbleController(XboxController xboxController) {
    if (getBlocked()){
      xboxController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
      System.out.println("rumbling");
    } else {
      xboxController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
      System.out.println("no rumble");
    }
  }

  public void stopRumble(XboxController xboxController){
    xboxController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }
  
  // public void setLights(){
  //   if (photoSensor.get()){
  //     candle.setLEDs(Constants.colorRGBIntake[0], Constants.colorRGBIntake[1], Constants.colorRGBIntake[2], 120, 0, 255);
  //   } else {
  //     candle.setLEDs(0, 0, 0);
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}