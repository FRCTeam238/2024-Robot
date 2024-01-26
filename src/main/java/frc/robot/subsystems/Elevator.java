package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Elevator.*;

public class Elevator extends SubsystemBase{
    CANSparkMax leadingMotor = new CANSparkMax(leaderId, MotorType.kBrushless);
    CANSparkMax followerMotor = new CANSparkMax(followerId, MotorType.kBrushless);
    public Elevator(){
        configureMotorController();
    }
    private void configureMotorController() {
        followerMotor.follow(leadingMotor);
        followerMotor.setSmartCurrentLimit(currentLimit);
        leadingMotor.setSmartCurrentLimit(currentLimit);
        leadingMotor.getForwardLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(false);
        leadingMotor.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(false);
        leadingMotor.getPIDController().setP(kP);
        leadingMotor.getPIDController().setI(kI);
        leadingMotor.getPIDController().setD(kD);
        leadingMotor.getPIDController().setOutputRange(-0.2, 1);
        
    }
}