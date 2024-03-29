package frc.robot.commands;

import static frc.robot.Constants.FeederConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Feeder;

public class EjectNote extends Command {
  private final Feeder feeder;

  public EjectNote() {
    this.feeder = Robot.feeder;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.feeder);
  }

  @Override
  public void initialize() {
    feeder.setCommand("EjectNote");
  }

  @Override
  public void execute() {
    feeder.rollerController(ejectPercent);
  }

  @Override
  public void end(boolean interrupted) {
    feeder.rollerController(0);
    feeder.setCommand("None");
  }
}
