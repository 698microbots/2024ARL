package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    //drive motors
    private TalonFX motor1 = new TalonFX(1);
    private TalonFX motor2 = new TalonFX(3);
    private TalonFX motor3 = new TalonFX(5);
    private TalonFX motor4 = new TalonFX(6);

    //turn motors
    private TalonFX Tmotor1 = new TalonFX(0);
    private TalonFX Tmotor2 = new TalonFX(2);
    private TalonFX Tmotor3 = new TalonFX(4);
    private TalonFX Tmotor4 = new TalonFX(7);    
    // private CurrentLimitsConfigs config1 = new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(50); // tis is used for current limiting
    // private CurrentLimitsConfigs config2 = new CurrentLimitsConfigs().withStatorCurrentLimit(30).withSupplyCurrentLimit(30); // tis is used for current limiting
 
    private CurrentLimitsConfigs config1 = new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentThreshold(2);
    private CurrentLimitsConfigs config2 = new CurrentLimitsConfigs().withStatorCurrentLimit(30).withSupplyCurrentThreshold(1);
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        // motor1.getConfigurator().apply(config1);
        // motor2.getConfigurator().apply(config1);
        // motor3.getConfigurator().apply(config1);
        // motor4.getConfigurator().apply(config1);
       
        // Tmotor1.getConfigurator().apply(config2);
        // Tmotor2.getConfigurator().apply(config2);
        // Tmotor3.getConfigurator().apply(config2);
        // Tmotor4.getConfigurator().apply(config2);   
        // config1.withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);   
        // config2.withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true);   

    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();        
        if (Utils.isSimulation()) {
            startSimThread();
        }
        config1.withStatorCurrentLimitEnable(true);
        config2.withStatorCurrentLimitEnable(true); //causes the current limits to set
        // ^^^ IMPORTANT
        motor1.getConfigurator().apply(config1);
        motor2.getConfigurator().apply(config1);
        motor3.getConfigurator().apply(config1);
        motor4.getConfigurator().apply(config1);      
       
        Tmotor1.getConfigurator().apply(config2);
        Tmotor2.getConfigurator().apply(config2);
        Tmotor3.getConfigurator().apply(config2);
        Tmotor4.getConfigurator().apply(config2); //also enables current limits
        
    }
    // TODO: try to add desaturate wheel speeds by adding a new SwerveDriveTrain
    // public void setControl(SwerveRequestMODIFIED request) {
    //     try {
    //         m_stateLock.writeLock().lock();

    //         m_requestToApply = request;
    //     } finally {
    //         m_stateLock.writeLock().unlock();
    //     }
    // }
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    // public Command applyRequest(Supplier<SwerveRequestMODIFIED> requestSupplier) {
    //     return run(() -> this.setControl(requestSupplier.get()));
    // }
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            ()->{
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              }, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements


    }    

    public Pose2d getPose(){
        return getState().Pose;
    }

    // public void resetPose(Pose2d pose){
    //     getState().Pose.
    // }
    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
        
    }

    public SwerveDriveKinematics getSwerveDriveKinematics(){
        return m_kinematics;
    }

    public String getSwerveModTarget(){
        String target = "";
        for (SwerveModule mod : Modules){
            target += mod.getTargetState().toString() + "/n";
        }
        return target;
    }
}
