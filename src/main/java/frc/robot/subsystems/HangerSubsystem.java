package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerSubsystem extends SubsystemBase {

    private final CANSparkMax HangerMotorOne = new CANSparkMax(0, CANSparkMax.MotorType.kBrushed);
    private final CANSparkMax HangerMotorTwo = new CANSparkMax(1, CANSparkMax.MotorType.kBrushed);

    public HangerSubsystem(){

    }

    // sets the motor speed for the Hanger
    public void setHangarMotorOne(double speed) {
        HangarMotorOne.set(speed * .1);
    }
    
    public void setHangarMotorTwo(double speed) {
        HangarMotorOne.set(speed * .1);        
    }
}
