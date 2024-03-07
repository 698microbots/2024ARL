package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlywheelSubsystem extends SubsystemBase{
    private final TalonFX motor1 = new TalonFX(Constants.flywheelMotor1); // create a new motor object
    private final TalonFX motor2 = new TalonFX(Constants.flywheelMotor2);
    private boolean scoreAmp = false;
    // constructor
    public FlywheelSubsystem(){

    }

    // sets the motor speed for the Flywheel
    public void setFlywheelMotorSpeed() {
        
        if(scoreAmp){
            motor1.set(Constants.ampFlywheelSpeed); //chage these to constants
            motor2.set(Constants.ampFlywheelSpeed);
        } else {
            motor1.set(Constants.speakerScoringSpeed);
            motor2.set(Constants.speakerScoringSpeed);
        }         
    }

    public void stopFlywheel() {
            motor1.set(0);
            motor2.set(0);
    }

    public void setScoringAmpFlywheel(boolean willScoreAmp){
        scoreAmp = willScoreAmp;
    }

    public boolean getScoringAmp(){
        return scoreAmp;
    }
}
