package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {

  CANSparkMax intakeMotor = new CANSparkMax(kID, MotorType.kBrushless);

  public Intake() {
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); //Motor position from internal encoder. Not currently used, 
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //Analog sensor. Not Used
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //Alternate Encoder. Not Used
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); //Absolute encoder position and angle
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //Absolute encoder velocity, not currently used, leave at default

    intakeMotor.setSmartCurrentLimit(40);
    Timer.delay(.02); //Pause between subsystems to ease CAN traffic at startup
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  @Log.NT
  public double getSpeed() {
    return intakeMotor.get();
  }
}
