package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotState;
import frc.robot.Robot;
import frc.robot.Utils;
import monologue.Annotations.Log;
import monologue.Logged;

public class Shooter extends SubsystemBase implements Logged {
  TalonFX leftMotor = new TalonFX(leftMotorId);
  TalonFX rightMotor = new TalonFX(rightMotorId);
  @Log double desiredLeftSpeed = 0;
  @Log double desiredRightSpeed = 0;
  @Log String command;

  public Shooter() {
    SmartDashboard.putNumber("LeftShooterSpeed", 4000);
    SmartDashboard.putNumber("RightShooterSpeed", 3800);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kV = kFF;
    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightMotor.setNeutralMode(NeutralModeValue.Coast);
    leftMotor.setNeutralMode(NeutralModeValue.Coast);
    leftMotor.getConfigurator().apply(config);
    rightMotor.getConfigurator().apply(config);
    leftMotor.setInverted(false);
    rightMotor.setInverted(true);

    rightMotor.getVelocity().setUpdateFrequency(100); // Set update frequency to 100 Hert, 10ms
    rightMotor.getClosedLoopError().setUpdateFrequency(50);
    rightMotor.getClosedLoopOutput().setUpdateFrequency(50);
    rightMotor.getSupplyVoltage().setUpdateFrequency(20);
    rightMotor.getSupplyCurrent().setUpdateFrequency(20);
    rightMotor.getStatorCurrent().setUpdateFrequency(20);
    rightMotor.optimizeBusUtilization();
    leftMotor.getVelocity().setUpdateFrequency(100); // Set update frequency to 100 Hert, 10ms
    leftMotor.getClosedLoopError().setUpdateFrequency(50);
    leftMotor.getClosedLoopOutput().setUpdateFrequency(50);
    leftMotor.getSupplyVoltage().setUpdateFrequency(20);
    leftMotor.getSupplyCurrent().setUpdateFrequency(20);
    leftMotor.getStatorCurrent().setUpdateFrequency(20);
    leftMotor.optimizeBusUtilization();

    Timer.delay(.02); // Pause between subsystems to ease CAN traffic at startup
  }

  public void setCommand(String name) {
    command = name;
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

  public void coast() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public void setWheelTargetSpeed() {
    switch (Robot.state) {
      case SUBWOOFER :
      case INTAKE :
        setSpeed(subwooferLeft, subwooferRight);
        break;
      case AMP :
        setSpeed(ampSpeed, ampSpeed);
        break;
      case PODIUM:
        setSpeed(podiumLeft, podiumRight);
      case TRAP :
        setSpeed(trapSpeed, trapSpeed);
        break;
      case DASHBOARD:
        setSpeed(SmartDashboard.getNumber("LeftShooterSpeed", 4000), SmartDashboard.getNumber("RightShooterSpeed",3800));
        break;
      case AUTO:
        //speed already set by spool and launch doesn't know the speed
        break;
      default:  //Targeting
        double distance = Utils.getSpeakerDistance();
        double avgSpeed = rpmTree.get(distance);
        setSpeed(avgSpeed + avgSpeed/speedDifference, avgSpeed - avgSpeed/speedDifference);
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

  @Log.NT
  public boolean isAtSpeed() {
    if (Math.abs(getLeftSpeed() - desiredLeftSpeed) < shooterTolerance)
    {
      if (Math.abs(getRightSpeed() - desiredRightSpeed) < shooterTolerance)
        return true;
      else
      {
        return false;
      }
    } else {
      return false;
    }
  }
}
