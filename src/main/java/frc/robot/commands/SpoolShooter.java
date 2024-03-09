// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SpoolShooter extends Command {
  /** Creates a new SpoolShooter. */
  public SpoolShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.shooter.setWheelTargetSpeed();
    Robot.shooter.setCommand("SpoolShooter");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Robot.shooter.coast();
    }
    Robot.shooter.setCommand("None");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.shooter.isAtSpeed();
  }
}
