package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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
    leftMotor.getConfigurator().apply(config);
    rightMotor.getConfigurator().apply(config);
  }

  public void setSpeed(double left, double right) {
    VelocityVoltage rightVelocityVoltage = new VelocityVoltage(right);
    VelocityVoltage leftVelocityVoltage = new VelocityVoltage(left);
    leftMotor.setControl(leftVelocityVoltage);
    rightMotor.setControl(rightVelocityVoltage);
    desiredLeftSpeed = left;
    desiredRightSpeed = right;
  }

  public double getLeftSpeed() {
    return leftMotor.getVelocity().getValueAsDouble();
  }

  public double getRightSpeed() {
    return rightMotor.getVelocity().getValueAsDouble();
  }

  boolean isAtSpeed() {
    if (getLeftSpeed() <= desiredLeftSpeed + 0.3 && getLeftSpeed() >= desiredLeftSpeed - 0.3) {
      return getRightSpeed() <= desiredRightSpeed + 0.3
          && getRightSpeed() > desiredRightSpeed - 0.3;
    } else {
      return false;
    }
  }
}
