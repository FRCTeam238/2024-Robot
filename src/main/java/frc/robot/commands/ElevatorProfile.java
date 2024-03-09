package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotionProfile;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class ElevatorProfile extends Command {
  private final Elevator elevator = Robot.elevator;
  private final MotionProfile.State goal;
  private final MotionProfile.MotionConstraints constraints;
  private MotionProfile profile;
  private MotionProfile.State currentState;

  public ElevatorProfile(MotionProfile.State goal, String name) {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.elevator);
    this.goal = goal;
    constraints =
        new MotionProfile.MotionConstraints(
            maxElevatorJerk, maxAccel, maxVelocity, velocityTolerance);
    setName(name);
  }

  @Override
  public void initialize() {

    currentState = new MotionProfile.State(elevator.getEncoderPosition(), elevator.getVelocity());

    profile = new MotionProfile(goal, currentState, constraints, MotionProfile.ProfileType.AUTO);
    elevator.setCommand(getName());
  }

  @Override
  public void execute() {
    if (goal.position < 6 && Robot.pivot.getCurrentPosition() > .53) {
      // Pivot is currently in position to collide with swerves, wait for it to clear
      // TODO: Any better way to handle this? Should these be made constants?
    } else {
      currentState = profile.sample();
      elevator.setDesiredState(currentState);
    }
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return Math.abs(elevator.getVelocity() - goal.velocity) <= velocityMaxError
        && Math.abs(elevator.getEncoderPosition() - goal.position) <= positionMaxError;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setCommand("None");
  }
}
