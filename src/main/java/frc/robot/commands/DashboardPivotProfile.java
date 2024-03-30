package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotionProfile;
import frc.robot.Robot;
import frc.robot.subsystems.Pivot;

import static frc.robot.Constants.PivotConstants.*;

public class DashboardPivotProfile extends Command {
  private final Pivot pivot;

  private MotionProfile profile;
  private MotionProfile.State goal;
  private MotionProfile.MotionConstraints constraints;

  public DashboardPivotProfile(String name) {
    pivot = Robot.pivot;
    constraints =
        new MotionProfile.MotionConstraints(maxJerk, maxAccel, maxVelocity, velocityTolerance);
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    setName(name);
    addRequirements(pivot);
    SmartDashboard.putNumber("PivotSetpoint", 0);
  }

  @Override
  public void initialize() {
    goal = new MotionProfile.State(SmartDashboard.getNumber("PivotSetpoint", 0));
    MotionProfile.State currentState =
        new MotionProfile.State(pivot.getCurrentPosition(), pivot.getVelocity());
    profile = new MotionProfile(goal, currentState, constraints, MotionProfile.ProfileType.AUTO);
    pivot.setCommand(getName());
  }

  @Override
  public void execute() {
    if (goal.position > .33 && Robot.elevator.getEncoderPosition() < 6) {
      // Pivot will collide with swerves, wait for elevator to go up
      // TODO: Better way to do this? Should these be constants?
    } else {
      MotionProfile.State sample = profile.sample();
      pivot.setDesiredState(sample);
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(pivot.getVelocity() - goal.velocity) <= velocityMaxError
        && Math.abs(pivot.getCurrentPosition() - goal.position) <= positionMaxError
        && profile.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setCommand("None");
  }
}
