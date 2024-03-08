// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Utils;

import static frc.robot.Constants.DriveConstants.*;

public class DriveToStageLeft extends Command {

  PIDController pidRotation, pidX, pidY;

  /** Creates a new TargetDT. */
  public DriveToStageLeft() {
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
    Robot.drivetrain.setCommand("DriveToStageLeft");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d stageLeft = Utils.stageLeft();
    Pose2d currentLocation = Robot.drivetrain.getPose();

    Robot.drivetrain.drive(
        pidX.calculate(currentLocation.getX(), stageLeft.getX()),
        pidY.calculate(currentLocation.getY(), stageLeft.getY()),
        pidRotation.calculate(currentLocation.getRotation().getRadians(), stageLeft.getRotation().getRadians())
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
