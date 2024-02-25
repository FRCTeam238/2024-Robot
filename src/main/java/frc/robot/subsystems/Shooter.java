package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils;
import monologue.Annotations.Log;
import monologue.Logged;

public class Shooter extends SubsystemBase implements Logged {
  TalonFX leftMotor = new TalonFX(0);
  TalonFX rightMotor = new TalonFX(0);
  @Log.NT double desiredLeftSpeed = 0;
  @Log.NT double desiredRightSpeed = 0;

  public Shooter() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kV = kFF;

    rightMotor.setInverted(true);
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
    double distance = Utils.getSpeakerDistance();
    double avgSpeed = rpmTree.get(distance);
    setSpeed(avgSpeed, avgSpeed); //TODO: find out if different sides should be different speeds?
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
    if (getLeftSpeed() <= desiredLeftSpeed + 0.3 && getLeftSpeed() >= desiredLeftSpeed - 0.3) {
      return getRightSpeed() <= desiredRightSpeed + 0.3
          && getRightSpeed() > desiredRightSpeed - 0.3;
    } else {
      return false;
    }
  }
}
