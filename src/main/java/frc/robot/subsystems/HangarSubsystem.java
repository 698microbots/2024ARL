package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangarSubsystem extends SubsystemBase{
    private final TalonFX Lmotor = new TalonFX(0); // create a new motor object (left)
    private final TalonFX Rmotor = new TalonFX(1); // create a new motor object (left)

    // constructor
    public HangarSubsystem(){

    }

    // sets the motor speed for the Flywheel
    public void setFlywheelMotorSpeed(double speed) {
        Lmotor.set(speed * 0.1);
        Rmotor.set(speed * 0.1);
     }
}
