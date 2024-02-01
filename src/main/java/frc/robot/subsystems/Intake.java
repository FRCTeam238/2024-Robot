package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {

  CANSparkMax intakeMotor = new CANSparkMax(kID, MotorType.kBrushless);

  public Intake() {}

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  @Log.NT
  public double getSpeed() {
    return intakeMotor.get();
  }
}
