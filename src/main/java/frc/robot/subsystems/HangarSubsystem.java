package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangarSubsystem extends SubsystemBase{


    // constructor
    public HangarSubsystem(){

    }

    // sets the motor speed for the Flywheel
    public void setFlywheelMotorSpeed(double speed) {
        Constants.Lmotor.set(speed * 0.1);
        Constants.Rmotor.set(speed * 0.1);
     }
}
