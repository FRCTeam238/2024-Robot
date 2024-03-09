package frc.robot.commands;

import static frc.robot.Constants.PivotConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotionProfile;
import frc.robot.Robot;
import frc.robot.subsystems.Pivot;

public class PivotProfile extends Command {
  private final Pivot pivot;

  private MotionProfile profile;
  private MotionProfile.State goal;
  private MotionProfile.MotionConstraints constraints;

  public PivotProfile(MotionProfile.State goal, String name) {
    pivot = Robot.pivot;
    this.goal = goal;
    constraints =
        new MotionProfile.MotionConstraints(maxJerk, maxAccel, maxVelocity, velocityTolerance);
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    setName(name);
    addRequirements(pivot);
  }

  @Override
  public void initialize() {

    MotionProfile.State currentState =
        new MotionProfile.State(pivot.getCurrentPosition(), pivot.getVelocity());
    profile = new MotionProfile(goal, currentState, constraints, MotionProfile.ProfileType.AUTO);
    pivot.setCommand(getName());
  }

  @Override
  public void execute() {
    if (goal.position > .53 && Robot.elevator.getEncoderPosition() < 6) {
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
        && Math.abs(pivot.getCurrentPosition() - goal.position) <= positionMaxError;
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setCommand("None");
  }
}
