package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangarSubsystem extends SubsystemBase {

    TalonFX Lmotor = new TalonFX(Constants.Lmotor);
    TalonFX Rmotor = new TalonFX(Constants.Rmotor);

    // constructor
    public HangarSubsystem(){

    }

    // sets the motor speed for the Flywheel
    public void setFlywheelMotorSpeed(double speed) {
        Lmotor.set(speed * 0.1);
        Rmotor.set(speed * 0.1);
     }
}
