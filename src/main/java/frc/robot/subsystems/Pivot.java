package frc.robot.subsystems;

import static frc.robot.Constants.PivotConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotionProfile;
import monologue.Annotations.Log;
import monologue.Logged;

public class Pivot extends SubsystemBase implements Logged {

  CANSparkMax pivotMotor = new CANSparkMax(pivotID, MotorType.kBrushless);
  AbsoluteEncoder encoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
  @Log String command;
  ArmFeedforward ff;

  public Pivot() {
    ff = new ArmFeedforward(kS, kG, kV);
    pivotMotor.getPIDController().setFeedbackDevice(encoder);
    SparkPIDController pidController = pivotMotor.getPIDController();

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setPositionPIDWrappingEnabled(false);
    pivotMotor.setSmartCurrentLimit(currentLimit);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setInverted(true); //set inversion such that CCW positive
    encoder.setPositionConversionFactor(
        2 * Math.PI * 31 / 74); // Convert rotations to rads then multiply by gearing
    encoder.setVelocityConversionFactor(
        (2 * Math.PI / 60) * 31 / 74); // Convert rotations to rads/s then multiply by gearing

    pivotMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, 100); // Motor position from internal encoder. Not currently used,
    pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Analog sensor. Not Used
    pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // Alternate Encoder. Not Used
    pivotMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus5, 10); // Absolute encoder position and angle
    pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10); // Absolute encoder velocity

    Timer.delay(.02); // Pause between subsystems to ease CAN traffic at startup
  }

  public void setCommand(String name) {
    command = name;
  }

  public void setDesiredState(MotionProfile.State state) {
    //Desired positions are angle of shooter side in radians with 0 equal horizontal, CCW positive = shooter up positive
    //Valid range should be ~70 degrees up to about 40 degrees down
    //Commanded postition translates this to the Spark encoder position where 0 is shooter straight up and CW positive
    //FF needs offset of Pi/2 as when at 0 in desired position from, CoG is actually straight up (Pi/2)
    double desiredPosition = state.position;
    double commandedPosition = -1*(desiredPosition-Math.PI/2);
    double desiredVelocity = state.velocity;
    double desiredAcceleration = state.acceleration;
    double feed = -1*ff.calculate(desiredPosition + Math.PI/2, desiredVelocity, desiredAcceleration); //flip FF because it wants CW positive

    this.log("desiredPosition", desiredPosition);
    this.log("commandedPosition", commandedPosition);
    this.log("desiredVelocity", desiredVelocity);
    this.log("desiredAccel", desiredAcceleration);
    this.log("feed", feed);

    pivotMotor
        .getPIDController()
        .setReference(commandedPosition, ControlType.kPosition, 0, feed, ArbFFUnits.kVoltage);
  }

  public void holdPosition() {
      pivotMotor.getPIDController().setReference(encoder.getPosition(), ControlType.kPosition, 0);
  }

  public void setSpeed(double speed) {
    pivotMotor.set(speed);
  }

  @Log.NT
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Log.NT
  public double getCurrentPosition() {
    return -(encoder.getPosition() - Math.PI/2);
  }

  @Log
  public double getSparkPosition() {
    return encoder.getPosition();
  };
  
  public void stop() {
    pivotMotor.set(0);
  }

}
