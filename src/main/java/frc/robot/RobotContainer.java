// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCenter;
import frc.robot.commands.AutoTest;
import frc.robot.commands.SetFlywheelMotor;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = .5; // 6 meters per second desired top speed (6 origin)
  private double MaxAngularRate = .5 * Math.PI; // 3/4 of a rotation per second max angular velocity (1.5 origin)
  public XboxController xboxController = new XboxController(0); // new XBox object

  // button definitions
  private final JoystickButton Xbutton = new JoystickButton(xboxController, Constants.Xbox_Button_X);
  private final JoystickButton Ybutton = new JoystickButton(xboxController, Constants.Xbox_Button_Y);
  private final JoystickButton Abutton = new JoystickButton(xboxController, Constants.Xbox_Button_A);
  private final JoystickButton Bbutton = new JoystickButton(xboxController, Constants.Xbox_Button_B);
  private final JoystickButton RBbutton = new JoystickButton(xboxController, Constants.Xbox_Button_RB);
  private final JoystickButton LBbutton = new JoystickButton(xboxController, Constants.Xbox_Button_LB);

  /* Setting up bindings for necessary control of the swerve drive platform */
  public final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private Pose2d pose = drivetrain.getState().Pose;

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    // Abutton.onTrue(new SetFlywheelMotor()); // tells the flywheel to move
    // to trigger a command to cernter the robot on an AprilTag, get the flywheel
    // and hanger in position
    Xbutton.onTrue(new AutoCenter());
    Xbutton.onTrue(new SetFlywheelMotor());

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    return new AutoTest(drivetrain, 1);
  }
}
