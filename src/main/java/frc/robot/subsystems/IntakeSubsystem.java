// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax IntakeMotor = new CANSparkMax(15, CANSparkMax.MotorType.kBrushless);
  // Initializes a DigitalInput on DIO 0
  private final DigitalInput photoSensor = new DigitalInput(1); //TODO - make this a constant
  private final CANdle candle = new CANdle(0);
  
  private boolean canRun = true;


  public IntakeSubsystem() {

  }

  public void setIntakeMotor(double speed) {
    if (getBlocked()) {
      IntakeMotor.set(0);
      // System.out.println("stopping motor");
    } else {
      IntakeMotor.set(-speed);
      // System.out.println("setting motor");
    
    }
  }

  //also use to override normal setIntakeMotor
  public void reverseIntakeMotor(double speed) {
    IntakeMotor.set(speed);
  }

  public void backupIntakeMotor(double speed){
    IntakeMotor.set(-speed);
  }
  public double getIntakeVolts() {
    return IntakeMotor.getBusVoltage();
  }

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
    return (!photoSensor.get());
  }

  public void rumbleController(XboxController xboxController, int value) {
    if (getBlocked()){
      xboxController.setRumble(GenericHID.RumbleType.kBothRumble, value);
    }
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