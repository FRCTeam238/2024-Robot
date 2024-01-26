package frc.robot.subsystems;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.signals.ControlModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.PivotConstants.*;
import frc.robot.MotionProfile;

public class Pivot extends SubsystemBase{

    CANSparkMax pivotMotor = new CANSparkMax(0, MotorType.kBrushless);
    AbsoluteEncoder encoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    
    ArmFeedforward ff; 

    public Pivot() {
        ff = new ArmFeedforward(0, 0, 0);
        pivotMotor.getPIDController().setFeedbackDevice(encoder);
        //TODO: set pid values
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
    
}
