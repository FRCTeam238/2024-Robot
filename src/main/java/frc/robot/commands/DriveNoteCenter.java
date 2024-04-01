package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OpInterface;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

/** Drive */
public class DriveNoteCenter extends Command {

  Drivetrain drivetrain = Robot.drivetrain;
  DoubleSubscriber txSub;
  PIDController pid;

  public DriveNoteCenter() {
    addRequirements(drivetrain);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    txSub = inst.getDoubleTopic("/limelight/tX").subscribe(0.0);
    pid = new PIDController(llkP, llkI, llkD);
    pid.setSetpoint(0);
  }

  @Override
  public void initialize() {
    drivetrain.setCommand("DriveNoteCenter");
  }

  @Override
  public void execute() {
    // if the |joystick x axis| is less than 10%, then set leftJoyX to 0

    double[] joyValues = OpInterface.getSwerveJoystickValues();

    //calculate robot relative speeds from joystick values
    ChassisSpeeds chassisSpeed = ChassisSpeeds.discretize(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        joyValues[0] * maxVelocityMetersPerSec, 
        joyValues[1] * maxVelocityMetersPerSec, 
        joyValues[2] * maxAngularVelocityRadsPerSec,
         drivetrain.getFieldRelativeOffset()), .02);

    //replace robot relative X with PID of limelight value
    chassisSpeed.vxMetersPerSecond = pid.calculate(txSub.get());

    drivetrain.driveWithChassisSpeeds(chassisSpeed);
  }
}
