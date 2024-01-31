package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.PivotConstants.*;


import frc.robot.MotionProfile;

public class Pivot extends SubsystemBase{

    CANSparkMax pivotMotor = new CANSparkMax(motor1, MotorType.kBrushless);
    AbsoluteEncoder encoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    
    ArmFeedforward ff; 

    public Pivot() {
        ff = new ArmFeedforward(kS, kG, kV);
        pivotMotor.getPIDController().setFeedbackDevice(encoder);
        //TODO: set pid values
        SparkPIDController pidController = pivotMotor.getPIDController();
                
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pivotMotor.setSmartCurrentLimit(currentLimit);
        pivotMotor.setIdleMode(IdleMode.kBrake);

    }

    public void setDesiredState(MotionProfile.State state) {
        double position = state.position;
        double velocity = state.velocity;
        double acceleration = state.acceleration;
        double feed = ff.calculate(position, velocity, acceleration);


        // -voltageMax < feed < voltageMax
        if (voltageMax < feed) {
            feed = voltageMax;
        } else if (-voltageMax > feed) {
            feed = -voltageMax;
        }
        


        pivotMotor.getPIDController().setReference(position, ControlType.kPosition, 0, feed, ArbFFUnits.kVoltage);
        
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getCurrentPosition(){
        return encoder.getPosition();
    }
}
