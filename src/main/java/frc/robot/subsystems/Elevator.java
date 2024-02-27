package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotionProfile;
import monologue.Annotations.Log;
import monologue.Logged;

public class Elevator extends SubsystemBase implements Logged {

  CANSparkMax leadingMotor = new CANSparkMax(leaderId, MotorType.kBrushless);
  CANSparkMax followerMotor = new CANSparkMax(followerId, MotorType.kBrushless);

  protected ElevatorFeedforward FF = new ElevatorFeedforward(kS, kG, kV, kA);

  public Elevator() {
    configureMotorController();
    Timer.delay(.02); //Pause between subsystems to ease CAN traffic at startup
  }

  public void setSpeed(double speed) {
    leadingMotor.set(speed);
  }

  private void configureMotorController() {
    followerMotor.follow(leadingMotor, true);
    followerMotor.setSmartCurrentLimit(currentLimit);
    leadingMotor.setSmartCurrentLimit(currentLimit);
    followerMotor.setIdleMode(IdleMode.kBrake);
    leadingMotor.setIdleMode(IdleMode.kBrake);
    leadingMotor.getForwardLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(false);
    leadingMotor.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(false);
    leadingMotor.setSoftLimit(SoftLimitDirection.kForward, softForwardLimit);
    leadingMotor.setSoftLimit(SoftLimitDirection.kReverse, softReverseLimit);
    leadingMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    leadingMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    leadingMotor.getPIDController().setP(kP);
    leadingMotor.getPIDController().setI(kI);
    leadingMotor.getPIDController().setD(kD);
    leadingMotor.getPIDController().setOutputRange(-0.2, 1);
    //Convert Spark unit (rotations) into inches of elevator travel
    leadingMotor.getEncoder().setPositionConversionFactor(inchesPerRev/gearing);
    leadingMotor.getEncoder().setVelocityConversionFactor((inchesPerRev/gearing)/60); //Native rotation unit is RPM, make In/s
    leadingMotor.getEncoder().setAverageDepth(2);
    leadingMotor.getEncoder().setMeasurementPeriod(16);

    leadingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5); //Applied output, used by follower
    leadingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10); //Motor velocity
    leadingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10); //Motor position from internal encoder.
    leadingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //Analog sensor. Not Used
    leadingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //Alternate Encoder. Not Used
    leadingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); //Absolute encoder position and angle
    leadingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //Absolute encoder velocity, not currently used, leave at default
    followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 50); //Applied output
    followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50); //Motor velocity
    followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50); //Motor position from internal encoder.
    followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //Analog sensor. Not Used
    followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //Alternate Encoder. Not Used
    followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); //Absolute encoder position and angle
    followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //Absolute encoder velocity, not currently used, leave at default

  }

  public void moveByPercentOutput(double percent) {
    leadingMotor.set(percent);
  }

  public void setDesiredState(MotionProfile.State currentState) {
    double feed = FF.calculate(currentState.velocity, currentState.acceleration);
    this.log("DesiredPosition", currentState.position);
    this.log("DesiredVelocity", currentState.velocity);
    this.log("DesiredAccel", currentState.acceleration);
    this.log("Feedforward", feed);
    leadingMotor
        .getPIDController()
        .setReference(currentState.position, ControlType.kPosition, 0, feed);
  }

  @Log.NT
  public double getVelocity() {
    return leadingMotor.getEncoder().getVelocity();
  }

  @Log.NT
  public double getEncoderPosition() {
    return leadingMotor.getEncoder().getPosition();
  }

  /**
   * gets the current velocity and encoder position as a {@link MotionProfile.State}
   */
  public MotionProfile.State getState() {
    return new MotionProfile.State(getEncoderPosition(), getVelocity());
  }
}
