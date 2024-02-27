package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.Constants.RobotState;
import monologue.Annotations.Log;
import monologue.Logged;

public class Shooter extends SubsystemBase implements Logged {
  TalonFX leftMotor = new TalonFX(leftMotorId);
  TalonFX rightMotor = new TalonFX(rightMotorId);
  @Log.NT double desiredLeftSpeed = 0;
  @Log.NT double desiredRightSpeed = 0;

  public Shooter() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kV = kFF;
    config.CurrentLimits.SupplyCurrentLimit = 30;

    rightMotor.setInverted(false);
    leftMotor.setInverted(true);
    rightMotor.setNeutralMode(NeutralModeValue.Coast);
    leftMotor.setNeutralMode(NeutralModeValue.Coast);
    leftMotor.getConfigurator().apply(config);
    rightMotor.getConfigurator().apply(config);

    rightMotor.getVelocity().setUpdateFrequency(100); //Set update frequency to 100 Hert, 10ms
    rightMotor.getClosedLoopError().setUpdateFrequency(50);
    rightMotor.getClosedLoopOutput().setUpdateFrequency(50);
    rightMotor.getSupplyVoltage().setUpdateFrequency(20);
    rightMotor.getSupplyCurrent().setUpdateFrequency(20);
    rightMotor.getStatorCurrent().setUpdateFrequency(20);
    rightMotor.optimizeBusUtilization();
    leftMotor.getVelocity().setUpdateFrequency(100); //Set update frequency to 100 Hert, 10ms
    leftMotor.getClosedLoopError().setUpdateFrequency(50);
    leftMotor.getClosedLoopOutput().setUpdateFrequency(50);
    leftMotor.getSupplyVoltage().setUpdateFrequency(20);
    leftMotor.getSupplyCurrent().setUpdateFrequency(20);
    leftMotor.getStatorCurrent().setUpdateFrequency(20);
    leftMotor.optimizeBusUtilization();

    Timer.delay(.02); //Pause between subsystems to ease CAN traffic at startup
  }

  public void setPercent(double left, double right) {
    leftMotor.set(left);
    rightMotor.set(right);
  }

  public void setSpeed(double left, double right) {
    VelocityVoltage rightVelocityVoltage = new VelocityVoltage(right);
    VelocityVoltage leftVelocityVoltage = new VelocityVoltage(left);
    leftMotor.setControl(leftVelocityVoltage);
    rightMotor.setControl(rightVelocityVoltage);
    desiredLeftSpeed = left;
    desiredRightSpeed = right;
  }

  public void coast(){
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public void setWheelTargetSpeed(){
    if(Robot.state == RobotState.SUBWOOFER)
    {
      setSpeed(subwooferLeft, subwooferRight);
    } else {
      double distance = Utils.getSpeakerDistance();
      double avgSpeed = rpmTree.get(distance);
      setSpeed(avgSpeed, avgSpeed); //TODO: find out if different sides should be different speeds?
    }
  }

  @Log.NT
  public double getLeftSpeed() {
    return leftMotor.getVelocity().getValueAsDouble();
  }

  @Log.NT
  public double getRightSpeed() {
    return rightMotor.getVelocity().getValueAsDouble();
  }

  public boolean isAtSpeed() {
    if (getLeftSpeed() <= desiredLeftSpeed + shooterTolerance && getLeftSpeed() >= desiredLeftSpeed - shooterTolerance) {
      return getRightSpeed() <= desiredRightSpeed + shooterTolerance
          && getRightSpeed() > desiredRightSpeed - shooterTolerance;
    } else {
      return false;
    }
  }
}
