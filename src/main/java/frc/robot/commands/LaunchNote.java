// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import org.frc238.lib.autonomous.AutonomousModeAnnotation;


@AutonomousModeAnnotation(parameterNames = {})
public class LaunchNote extends Command {
  /** Creates a new LaunchNode. */
  public LaunchNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.feeder, Robot.shooter);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.feeder.rollerController(1);
    Robot.shooter.setWheelTargetSpeed();
    Robot.feeder.setCommand("LaunchNote");
    Robot.shooter.setCommand("LaunchNote");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.feeder.rollerController(0);
    Robot.shooter.coast();
    Robot.feeder.setCommand("None");
    Robot.shooter.setCommand("None");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
