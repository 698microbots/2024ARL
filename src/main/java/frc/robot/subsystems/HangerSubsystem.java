package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerSubsystem extends SubsystemBase {

    private final CANSparkMax HangerMotorOne = new CANSparkMax(16, CANSparkMax.MotorType.kBrushed);
    private final CANSparkMax HangerMotorTwo = new CANSparkMax(17, CANSparkMax.MotorType.kBrushed);

    public HangerSubsystem(){

    }

    // sets the motor speed for the Hanger
    public void setHangerMotorOne(double speed) {
        HangerMotorOne.set(speed);
    }
    
    public void setHangerMotorTwo(double speed) {
        HangerMotorTwo.set(speed);        
    }
}
