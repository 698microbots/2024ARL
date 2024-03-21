// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static double numSeconds(double seconds){ //does the calculations for how many 20ms are in a second, compare it to a counter adds every 20ms
      return seconds * 50;
  }


  // // motor constants
  // public static final int Lmotor = 8; // create a new motor object (left)
  // public static final int Rmotor = 9; // create a new motor object (left)
  // public static final int armMotor = 12;
  // public static final int motor1 = 13;
  // public static final int motor2 = 14;

  // Controller Button IDs
  public static final int Xbox_Button_A = 1;
  public static final int Xbox_Button_B = 2;
  public static final int Xbox_Button_X = 3;
  public static final int Xbox_Button_Y = 4;
  public static final int Xbox_Button_LB = 5;
  public static final int Xbox_Button_RB = 6;
  public static final int Xbox_Button_LS = 9;
  public static final int Xbox_Button_RS = 10;

  // controller trigger IDs
  public static final int XBox_Trigger_L = 2;
  public static final int XBox_Trigger_R = 3;

  // constants for the LimeLight
  // public static final int goalHeight = 0; // get this from the game manual
  // later // shouldn't be needed
  public static final int limeLightHeight = 0/* placeholder */;
  public static final double limeLightInitAngle = 0/* placeholder */;
  public static final double goalHeight = 0.0;

  public static final double angleToRadians = (Math.PI / 180);
  public static final double radiansToAngle = (180 / Math.PI);

  // values turn motors are 7, 2, 0, 4
  public static final int frontLeftDrive = 6;
  public static final int frontLeftTurn = 7;
  public static final int frontLeftCanCoder = 0; // the cancoder ones are probably not right

  public static final int frontRightDrive = 1;
  public static final int frontRightTurn = 2;
  public static final int frontRightCanCoder = 1; // the cancoder ones are probably not right

  public static final int backLeftDrive = 3; // 5
  public static final int backLeftTurn = 4; // 0
  public static final int backLeftCanCoder = 3; // the cancoder ones are probably not right

  public static final int backRightDrive = 5; // 3
  public static final int backRightTurn = 0;  //4
  public static final int backRightCanCoder = 2; //the cancoder ones are probably not right

  public static final int flywheelMotor1 = 8;
  public static final int flywheelMotor2 = 9;
  public static final int armMotor = 10;
  public static final int armMotor2 = 11;

  public static final double speakerTagHeightMeters = 0;
  public static final double ampTagHeightMeters = 0;
  public static final double endArmPositionSpeed = 0.05;
  public static final double ampEncoderUnits = 0;
  public static final double intakeNoteVoltage = 11.5;
  public static final double encoderTrap = .33;
  public static final double noteAreaToRun = 2.5;
  public static final double ampFlywheelSpeed = -.5;
  public static final double speakerScoringSpeed = -1;
  public static final int[] colorRGBIntake = {255, 153, 51};
}
