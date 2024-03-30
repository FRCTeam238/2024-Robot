package frc.robot.commands;

import static frc.robot.Constants.PivotConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotionProfile;
import frc.robot.Robot;
import frc.robot.subsystems.Pivot;

public class PivotUnprofiled extends Command {
  private final Pivot pivot;

  private MotionProfile profile;
  private MotionProfile.State goal;
  private MotionProfile.MotionConstraints constraints;

  public PivotUnprofiled(MotionProfile.State goal, String name) {
    pivot = Robot.pivot;
    this.goal = goal;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    setName(name);
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    pivot.setCommand(getName());
  }

  @Override
  public void execute() {
    if (goal.position > .33 && Robot.elevator.getEncoderPosition() < 6) {
      // Pivot will collide with swerves, wait for elevator to go up
      // TODO: Better way to do this? Should these be constants?
    } else {
      pivot.setDesiredState(goal);
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
