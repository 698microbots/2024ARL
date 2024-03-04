// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax IntakeMotor = new CANSparkMax(15, CANSparkMax.MotorType.kBrushless);
  // Initializes a DigitalInput on DIO 0
  private final DigitalInput photoSensor = new DigitalInput(2); //TODO - make this a constant
  
  private boolean canRun = true;

  public IntakeSubsystem() {
  }

  public void setIntakeMotor(double speed) {
    if(canRun){
    IntakeMotor.set(-speed);
    } else {
      IntakeMotor.set(0);
    }
  }

  public void reverseIntakeMotor(double speed){
    IntakeMotor.set(speed);
  }

  public double getIntakeVolts() {
    return IntakeMotor.getBusVoltage();
  }

  public void setCanRun(boolean run) {
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
    return photoSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
