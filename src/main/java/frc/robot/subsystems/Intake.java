package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  CANSparkMax intakeMotor = new CANSparkMax(kID, MotorType.kBrushless);

  public Intake() {}

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }
}
