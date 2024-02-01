package frc.robot.subsystems;

import static frc.robot.Constants.PivotConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotionProfile;
import monologue.Annotations.Log;
import monologue.Logged;

public class Pivot extends SubsystemBase implements Logged {

  CANSparkMax pivotMotor = new CANSparkMax(motor1, MotorType.kBrushless);
  AbsoluteEncoder encoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

  ArmFeedforward ff;

  public Pivot() {
    ff = new ArmFeedforward(kS, kG, kV);
    pivotMotor.getPIDController().setFeedbackDevice(encoder);
    SparkPIDController pidController = pivotMotor.getPIDController();

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pivotMotor.setSmartCurrentLimit(currentLimit);
    pivotMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setDesiredState(MotionProfile.State state) {
    double desiredPosition = state.position;
    double desiredVelocity = state.velocity;
    double desiredAcceleration = state.acceleration;
    double feed = ff.calculate(desiredPosition, desiredVelocity, desiredAcceleration);
    this.log("desiredPosition", desiredPosition);
    this.log("desiredVelocity", desiredVelocity);
    this.log("desiredAccel", desiredAcceleration);
    this.log("feed", feed);

    // -voltageMax < feed < voltageMax
    if (voltageMax < feed) {
      feed = voltageMax;
    } else if (-voltageMax > feed) {
      feed = -voltageMax;
    }

    pivotMotor
        .getPIDController()
        .setReference(desiredPosition, ControlType.kPosition, 0, feed, ArbFFUnits.kVoltage);
  }

  @Log.NT
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Log.NT
  public double getCurrentPosition() {
    return encoder.getPosition();
  }
}
