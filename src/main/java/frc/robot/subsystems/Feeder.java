// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.Constants.FeederConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  CANSparkMax feederMotor = new CANSparkMax(deviceId, MotorType.kBrushless);
  DigitalInput sensorMachine = new DigitalInput(deviceId);
  public Feeder() {}

  public void rollerController(double speed){
    feederMotor.set(speed);
  }
  public boolean sensor(){
    return sensorMachine.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
