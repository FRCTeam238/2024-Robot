package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MotionProfile;
import frc.robot.Robot;
import frc.robot.subsystems.Pivot;

public class VisionPivot extends Command {
  private final Pivot pivot;
  private final PIDController controller;

 

  public VisionPivot(Supplier<MotionProfile.State> goal, String name) {
    pivot = Robot.pivot;
    controller = new PIDController(0, 0, 0);

    
    // addRequirements() method (which takes a vararg of Subsystem)
    setName(name);
    addRequirements(pivot);
  }

  @Override
  public void initialize() {

    
  }

  @Override
  public void execute() {
    // if (Robot.vision.)
    // pivot.setSpeed(controller.calculate(pivot.getCurrentPosition(), ));
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setCommand("None");
  }

  public boolean onTarget() {
    return false;
  }
}
