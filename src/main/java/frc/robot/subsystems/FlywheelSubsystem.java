package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase{
    private final TalonFX motor = new TalonFX(0); // create a new motor object

    // constructor
    public FlywheelSubsystem(){

    }

    // sets the motor speed for the Flywheel
    public void setFlywheelMotorSpeed(double speed) {
        motor.set(speed);
        }
}
