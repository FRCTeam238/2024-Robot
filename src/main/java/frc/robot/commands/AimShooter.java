// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotionProfile;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.MotionProfile.MotionConstraints;
import frc.robot.MotionProfile.ProfileType;
import frc.robot.MotionProfile.State;

public class AimShooter extends Command {

  double desiredElevatorHeight;
  double desiredPivotAngle;

  /** Creates a new AimShooter. */
  public AimShooter() {
    addRequirements(Robot.elevator, Robot.pivot, Robot.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = Utils.getSpeakerDistance();
    //Here we need to do math to make distance into height, pivot angle and wheel speed. Store elevator and pivot in class fields.

    MotionProfile elevatorProfile = new MotionProfile(
      new State(desiredElevatorHeight), 
      new State(Robot.elevator.getEncoderPosition(), Robot.elevator.getVelocity()), 
      new MotionConstraints(0, 0, 0, 0), //TODO: Replace with constants from elevator section
      ProfileType.AUTO);
    Robot.elevator.setDesiredState(elevatorProfile.sample());

    MotionProfile pivotProfile = new MotionProfile(
      new State(desiredPivotAngle), 
      new State(Robot.pivot.getCurrentPosition(), Robot.pivot.getVelocity()), 
      new MotionConstraints(0, 0, 0, 0), //TODO: Replace with constants from pivot section
      ProfileType.AUTO);
    Robot.pivot.setDesiredState(pivotProfile.sample());

    Robot.shooter.setWheelTargetSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean elevatorOnTarget = Math.abs(desiredElevatorHeight - Robot.elevator.getEncoderPosition()) < ElevatorConstants.elevatorTolerance;
    boolean pivotOnTarget = Math.abs(desiredPivotAngle - Robot.pivot.getCurrentPosition()) < PivotConstants.pivotTolerance;
    return elevatorOnTarget && pivotOnTarget && Robot.shooter.isAtSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.shooter.setSpeed(0, 0);
  }

}
