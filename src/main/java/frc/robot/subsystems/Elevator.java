package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.math.controller.ElevatorFeedforward;
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
