package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OpInterface;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

/** Drive */
public class Drive extends Command {

  Drivetrain drivetrain = Robot.drivetrain;

  public Drive() {
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setCommand("Drive");
  }

  @Override
  public void execute() {
    // if the |joystick x axis| is less than 10%, then set leftJoyX to 0

    double[] joyValues = OpInterface.getSwerveJoystickValues();

    drivetrain.drive(
        joyValues[0] * maxVelocityMetersPerSec,
        joyValues[1] * maxVelocityMetersPerSec,
        joyValues[2] * maxAngularVelocityRadsPerSec);
  }
}
