// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AUTOTESTIntakeMoveAndDriveTrain;
import frc.robot.commands.AUTOTESTarmDown;
import frc.robot.commands.AUTOTESTautoArmShoot;
import frc.robot.commands.AUTOTESTmove;
import frc.robot.commands.AUTOTESTtagAutoShoot;
import frc.robot.commands.AUTOTESTtagAutoShoot;
import frc.robot.commands.AutoCenterAmp;
import frc.robot.commands.AutoCenterSpeaker;
import frc.robot.commands.AutoScoreSpeakerArm;
import frc.robot.commands.AutoSetLEDS;
import frc.robot.commands.AutoTrap;
import frc.robot.commands.AutoTrapFromGround;
import frc.robot.commands.BackUpIntake;
import frc.robot.commands.FlyWheelShoot;
import frc.robot.commands.FlywheelShootAmp;
import frc.robot.commands.IntakeMove;
import frc.robot.commands.MoveHanger;
import frc.robot.commands.TESTMoveArm;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.driveTrainVoltages;

public class RobotContainer {
  private double MaxSpeed = 4; // 6 meters per second desired top speed (6 origin)
  private double MaxAngularRate = 1.3 * Math.PI; // 3/4 of a rotation per second max angular velocity (1.5 origin)
  public XboxController xboxController = new XboxController(0); // new XBox object
  public XboxController xboxController2 = new XboxController(1); // new XBox objec
  SlewRateLimiter slewRateDriveX = new SlewRateLimiter(.6); //old: .5
  SlewRateLimiter slewRateDriveY = new SlewRateLimiter(.6); //old: .5
  SlewRateLimiter slewRateTurn = new SlewRateLimiter(.6);
  
  /*
   * 
   * TODO: MaxAngularRate really effects driving in a straight line, if its too slow then swerve will drift off to the side in which its turning
   * Might have to desaturate wheel speeds in the SwerveRequest Class
   */

  // button definitions
  // private final JoystickButton Xbutton = new JoystickButton(xboxController, Constants.Xbox_Button_X);
  // private final JoystickButton Ybutton = new JoystickButton(xboxController, Constants.Xbox_Button_Y);
  // private final JoystickButton Abutton = new JoystickButton(xboxController, Constants.Xbox_Button_A);
  // private final JoystickButton Bbutton = new JoystickButton(xboxController, Constants.Xbox_Button_B);
  // private final JoystickButton RBbutton = new JoystickButton(xboxController, Constants.Xbox_Button_RB);
  // private final JoystickButton LBbutton = new JoystickButton(xboxController, Constants.Xbox_Button_LB);

  // private final JoystickButton Xbutton2 = new JoystickButton(xboxController2, Constants.Xbox_Button_X);
  // private final JoystickButton Ybutton2 = new JoystickButton(xboxController2, Constants.Xbox_Button_Y);
  // private final JoystickButton Abutton2 = new JoystickButton(xboxController2, Constants.Xbox_Button_A);
  // private final JoystickButton Bbutton2 = new JoystickButton(xboxController2, Constants.Xbox_Button_B);
  // private final JoystickButton RBbutton2 = new JoystickButton(xboxController2, Constants.Xbox_Button_RB);
  // private final JoystickButton LBbutton2 = new JoystickButton(xboxController2, Constants.Xbox_Button_LB);
  
  /* Setting up bindings for necessary control of the swerve drive platform */
  public LimeLightSubsystem limeLight = new LimeLightSubsystem();
  public final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandXboxController joystick2 = new CommandXboxController(1);
  public final ArmSubsystem arm = new ArmSubsystem();
  public final HangerSubsystem hanger = new HangerSubsystem();
  public FlywheelSubsystem flyWheel = new FlywheelSubsystem();
  public GyroSubsystem gyro = new GyroSubsystem();
  public Telemetry telemetry = new Telemetry(3.5);
  public driveTrainVoltages driveTrainVoltages = new driveTrainVoltages();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final LightSubsystem lights = new LightSubsystem();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  public Pose2d pose = drivetrain.getState().Pose; //could break the code 
  public final Field2d field2d = new Field2d();

  private Command runAuto = drivetrain.getAutoPath("Another Auto Test");

  private SwerveModuleState[] states = drivetrain.getState().ModuleStates;

  private void configureBindings() {
    lights.setDefaultCommand(new AutoSetLEDS(lights));
    //drive command
    // flyWheel.setDefaultCommand(new FlyWheelShoot(flyWheel, limeLight, intake, () -> joystick.getLeftTriggerAxis()));
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(slewRateDriveX.calculate(-joystick.getLeftY()) * MaxSpeed) // Drive forward with
            // negative Y (forward)
            .withVelocityY(slewRateDriveY.calculate(-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(slewRateTurn.calculate(-joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));


    //brake mode
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    //point wheels
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
    //fixed trap angle
    joystick.b().whileTrue(new AutoScoreSpeakerArm(arm, flyWheel, intake));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //backup intake
    joystick.x().whileTrue(new BackupIntake(intake));

    //speaker score
    joystick.leftTrigger().whileTrue(new FlyWheelShoot(flyWheel, intake, xboxController, xboxController2, () -> joystick.getLeftTriggerAxis()));
    
    //amp score
    joystick.rightTrigger().whileTrue(new FlywheelShootAmp(flyWheel, intake));

    //auto trap
    // joystick.y().whileTrue(new AutoTrap(flyWheel, intake, arm));
    // joystick.y().whileTrue(new AutoTrapFromGround(intake, flyWheel, limeLight, drivetrain, arm));
      joystick.y().whileTrue(
           new AutoCenterSpeaker(
              () -> joystick.getLeftX() * MaxSpeed,
              () -> joystick.getLeftY() * MaxSpeed,
              drivetrain,
              limeLight,
              MaxAngularRate,
              arm,
              flyWheel,
              intake)
               
      );    /**
     * 
     * 2nd driver commands
     * 
     */
    //hanger commands
    hanger.setDefaultCommand(new MoveHanger(
        hanger, 
        () -> joystick2.getLeftTriggerAxis(), 
        () -> joystick2.getRightTriggerAxis(), 
        () -> xboxController2.getLeftBumper(), 
        () -> xboxController2.getRightBumper()));


    //default arm command, move it with 2nd controller    
    arm.setDefaultCommand(new TESTMoveArm(arm, () -> joystick2.getLeftY() * .6));
    arm.setDefaultCommand(new TESTMoveArm(arm, () -> joystick2.getLeftY() * .6));
    
    //reverse intake
    joystick2.a().whileTrue(new IntakeMove(xboxController, xboxController2, intake, limeLight, true, lights));

    // //auto amp sequence to move up to the amp and arm
    joystick2.y().whileTrue(new AutoCenterAmp(drivetrain, () -> joystick2.getLeftY(), () -> joystick2.getLeftX(), MaxSpeed, limeLight));

    //auto center with speaker and move arm accordingly
      joystick2.x().whileTrue(
           new AutoCenterSpeaker(
              () -> joystick.getLeftX() * MaxSpeed,
              () -> joystick.getLeftY() * MaxSpeed,
              drivetrain,
              limeLight,
              MaxAngularRate,
              arm,
              flyWheel,
              intake)
               
      );

    // //auto center with note and run intake when close enough, this is probably gonna have CAN bad errors
      //  joystick2.b().whileTrue(new AutoCenterNoteAndIntake(
      //   () -> joystick.getLeftX() * MaxSpeed, 
      //   () -> joystick.getLeftY() * MaxSpeed, 
      //   drivetrain,
      //   limeLight,
      //   MaxAngularRate, 
      //   intake, 
      //   lights, 
      //   xboxController2, 
      //   xboxController));
      joystick2.b().whileTrue(
        new IntakeMove(
          xboxController, 
          xboxController2, 
          intake, limeLight,
          false, 
          lights,
          drivetrain,
          () -> joystick.getLeftX(),
          () -> joystick.getLeftY(),
          () -> joystick.getRightX())
      );

    ////////////////////////////////////////////////////////////////////////////////

    // Xbutton.onTrue(new SequentialCommandGroup(new AutoCenter(), new SetFlywheelMotor()));

    // flyWheel.setDefaultCommand(new TESTFlywheel(flyWheel, () -> joystick2.getLeftY() * .85));    
    // joystick.x().whileTrue(new AutoCenter(drivetrain, limeLight, MaxAngularRate));
    // joystick.b().whileTrue(new AutoCenterNote(() -> joystick.getLeftX() * MaxSpeed, () -> joystick.getLeftY() * MaxSpeed, drivetrain, limeLight));    
    //RECOMENT ABOCE IF NOT WORKING
    //TODO: this creates really weird can disabled errors for some reason: parallel command groups cannot use 2 of the same subsystems
    // joystick.y().whileTrue(new ParallelCommandGroup(
    //   new AutoCenterNote(() -> joystick.getLeftX() * MaxSpeed, () -> joystick.getLeftY() * MaxSpeed, drivetrain, limeLight),
    //   new IntakeMove(intake, limeLight)
    // ));
    // joystick.y().whileTrue(new TESTBrokenButtons());
    // joystick2.b().whileTrue(new MoveHanger(false, true, hanger));
    // joystick.y().whileTrue(new AutoCenterNoteAndIntake(() -> joystick.getLeftX() * MaxSpeed,
    //     () -> joystick.getLeftY() * MaxSpeed, drivetrain, limeLight, MaxSpeed, intake));
    // joystick2.x().whileTrue(new SetAutoArm(arm));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    NamedCommands.registerCommand("IntakeMove", new IntakeMove(xboxController, xboxController2, intake, limeLight, false, lights));
    NamedCommands.registerCommand("AUTOTESTarmDown", new AUTOTESTarmDown(arm));
    
    configureBindings();

  }

  public Command getAutonomousCommand() {

    // return new SequentialCommandGroup( //auto seq one
    //   new AUTOTESTautoArmShoot(arm, flyWheel, intake, limeLight, drivetrain, 2),
    //   new AUTOTESTmove(drivetrain, 1, 1, 0, 1),
    //   new AUTOTESTIntakeMoveAndDriveTrain(intake, drivetrain, 2, 1, 0 , 0)
    // );

    //(0 note) move out of alliance 
    // return new SequentialCommandGroup(
    //   new AUTOTESTmove(drivetrain, 2, -1, 0, 0)
    // );

    //(2 note) ***USING shoots note infront of speaker drives back picks up note and shoots again,then moves out of alliance
    return new SequentialCommandGroup(
      new AUTOTESTautoArmShoot(arm, flyWheel, intake, limeLight, drivetrain, 2), //try changing this to 0
      new AUTOTESTarmDown(arm),
      new AUTOTESTIntakeMoveAndDriveTrain(intake, drivetrain, 1.75, 1, 0, 0),
      new AUTOTESTmove(drivetrain, 1.99, -1, 0, 0), 
      new AUTOTESTautoArmShoot(arm, flyWheel, intake, limeLight, drivetrain, 2),
      new AUTOTESTarmDown(arm),
      new AUTOTESTmove(drivetrain, 2.5, 1, 0, 0)
    );

    //(1 note)
    // return new SequentialCommandGroup(
    //   new AUTOTESTautoArmShoot(arm, flyWheel, intake, limeLight, drivetrain, 2),
    //   new AUTOTESTarmDown(arm),
    //   new AUTOTESTmove(drivetrain, 2, 1, 0, 0)
    // );

    // //(2 note) shoots note picks up another note and shots again, amp side
    // return new SequentialCommandGroup(
    //   new AUTOTESTautoArmShoot(arm, flyWheel, intake, limeLight, drivetrain, 2),
    //   new AUTOTESTarmDown(arm),
    //   new AUTOTESTmove(drivetrain, 1, 0, 1, Math.PI * .5),
    //   new 
    // );

    // 3 note prayge BLUE ALLAINCE
    // return new SequentialCommandGroup(
    //   new AUTOTESTautoArmShoot(arm, flyWheel, intake, limeLight, drivetrain, 2), //try changing this to 0
    //   new AUTOTESTarmDown(arm),
    //   new AUTOTESTIntakeMoveAndDriveTrain(intake, drivetrain, 1.75, 1, 0, 0),
    //   new AUTOTESTtagAutoShoot(limeLight, intake, arm, drivetrain, flyWheel),
    //   new AUTOTESTIntakeMoveAndDriveTrain(intake, drivetrain, 1.75, 0, 1, Math.PI * .125), // this part determines 3 note
    //   new AUTOTESTtagAutoShoot(limeLight, intake, arm, drivetrain, flyWheel)
    // );

    // 3 note prayge RED ALLAINCE
    // return new SequentialCommandGroup(
    //   new AUTOTESTautoArmShoot(arm, flyWheel, intake, limeLight, drivetrain, 2), //try changing this to 0
    //   new AUTOTESTarmDown(arm),
    //   new AUTOTESTIntakeMoveAndDriveTrain(intake, drivetrain, 1.75, 1, 0, 0),
    //   new AUTOTESTtagAutoShoot(limeLight, intake, arm, drivetrain, flyWheel),
    //   new AUTOTESTIntakeMoveAndDriveTrain(intake, drivetrain, 1.75, 0, 1, Math.PI * .125),
    //   new AUTOTESTtagAutoShoot(limeLight, intake, arm, drivetrain, flyWheel)
    // );


    // return Commands.print("No autonomous command configured");
    // PathPlannerPath path = PathPlannerPath.fromPathFile("New Auto");
    // // return new TESTauto(drivetrain, 5);
    // // return drivetrain.applyRequest(null);
    // return AutoBuilder.followPath(path);
    // return runAuto;    
  }
}
