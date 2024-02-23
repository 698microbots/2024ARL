// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
    private final CANSparkMax IntakeMotor = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);
    private boolean canRun = false;
  public IntakeSubsystem() {}

  public void setIntakeMotor(double speed){
    IntakeMotor.set(speed * .5);
  }

  public void setCanRun(boolean run){
    canRun = run;
  }

  public boolean getCanRun(){
    return canRun;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
