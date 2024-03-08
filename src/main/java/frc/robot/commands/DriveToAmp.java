// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Utils;

public class DriveToAmp extends Command {

  PIDController pidRotation, pidX, pidY; 

  /** Creates a new TargetDT. */
  public DriveToAmp() {
    addRequirements(Robot.drivetrain);
    pidRotation = new PIDController(kPAngular, kIAngular, kDAngular);
    pidRotation.setTolerance(turnTolerance, velocityTolerance);
    pidX = new PIDController(kP, kI, kD);
    pidY = new PIDController(kP, kI, kD);
    pidX.setTolerance(positionTolerance, xandyvelocityTolerance);
    pidY.setTolerance(positionTolerance, xandyvelocityTolerance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drivetrain.setCommand("DriveToAmp");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d ampLocation = Utils.ampLocation();
    Pose2d currentLocation = Robot.drivetrain.getPose();

    Robot.drivetrain.drive(
        pidX.calculate(currentLocation.getX(), ampLocation.getX()),
        pidY.calculate(currentLocation.getY(), ampLocation.getY()),
        pidRotation.calculate(currentLocation.getRotation().getRadians(), ampLocation.getRotation().getRadians())
    );

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidRotation.atSetpoint() && pidX.atSetpoint() && pidY.atSetpoint();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.drivetrain.drive(0, 0, 0);
  }
}
