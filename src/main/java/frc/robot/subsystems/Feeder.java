// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.FeederConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Feeder extends SubsystemBase implements Logged {
  /** Creates a new Feeder. */
  CANSparkMax feederMotor = new CANSparkMax(feederId, MotorType.kBrushless);

  DigitalInput sensorMachine = new DigitalInput(sensorId);
  @Log String command;

  public Feeder() {
    feederMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, 65535); // Motor position from internal encoder. Not used
    feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // Analog sensor. Not Used
    feederMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus4, 65535); // Alternate Encoder. Not Used
    feederMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus5, 65535); // Absolute encoder position and angle. Not used
    feederMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus6, 65535); // Absolute encoder velocity, not used

    feederMotor.setSmartCurrentLimit(30);
    Timer.delay(.02); // Pause between subsystems to ease CAN traffic at startup
    feederMotor.setInverted(true);
  }

  public void setCommand(String name) {
    command = name;
  }

  public void rollerController(double speed) {
    feederMotor.set(speed);
  }

  public void stop() {
    feederMotor.set(0);
  }

  @Log.NT
  public boolean sensor() {
    return !sensorMachine.get();
  }

  @Log.NT
  public double getSpeed() {
    return feederMotor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
