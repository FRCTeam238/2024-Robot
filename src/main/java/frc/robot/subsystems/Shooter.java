package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter {
    TalonFX leftMotor = new TalonFX(0);    
    TalonFX rightMotor = new TalonFX(0); 
    double desiredLeftSpeed = 0;
    double desiredRightSpeed = 0;
    
    public Shooter() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kFF;

        rightMotor.setInverted(true);

    }
    public void setSpeed(double left, double right) {
        leftMotor.set(left);
        rightMotor.set(right);
        desiredLeftSpeed = left;
        desiredRightSpeed = right;
    }

    public double getLeftSpeed() {
        return leftMotor.getVelocity().getValueAsDouble();
    }
    public double getRightSpeed() {
    return rightMotor.getVelocity().getValueAsDouble();
    }
    boolean isAtSpeed(){
        if(getLeftSpeed() <= desiredLeftSpeed + 0.3 && getLeftSpeed() >= desiredLeftSpeed - 0.3){
            if(getRightSpeed() <= desiredRightSpeed + 0.3 && getRightSpeed() > desiredRightSpeed - 0.3){
                return true;
            }
            return false;
        }else{
            return false;
        }

    }



}
