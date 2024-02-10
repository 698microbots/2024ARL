package frc.robot.subsystems;

import java.io.Console;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlywheelSubsystem extends SubsystemBase{
    private final TalonFX motor1 = new TalonFX(Constants.motor1); // create a new motor object
    private final TalonFX motor2 = new TalonFX(Constants.motor2);
    // constructor
    public FlywheelSubsystem(){

    }

    // sets the motor speed for the Flywheel
    public void setFlywheelMotorSpeed(double speed) {
        motor1.set(speed);
        motor2.set(speed);
        }


}
