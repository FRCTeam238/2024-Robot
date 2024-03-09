// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.Utils;

public class AimDT extends Command {

  PIDController pid;

  /** Creates a new TargetDT. */
  public AimDT() {
    addRequirements(Robot.drivetrain);
    pid = new PIDController(kPAngular, kIAngular, kDAngular);
    pid.setTolerance(turnTolerance, velocityTolerance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drivetrain.setCommand("AimDT");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] joyValues = OI.getSwerveJoystickValues();

    double speakerLocation = getSpeakerAngle();
    double robotAngle = Robot.drivetrain.getPose().getRotation().getRadians();

    Robot.drivetrain.drive(
        joyValues[0] * maxVelocityMetersPerSec,
        joyValues[1] * maxVelocityMetersPerSec,
        pid.calculate(robotAngle, speakerLocation));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  private double getSpeakerAngle() {
    Pose2d currentPosition = Robot.drivetrain.getPose();
    Pose2d speakerLocation = Utils.speakerLocation();
    return currentPosition.minus(speakerLocation).getRotation().getRadians();
  }
}
