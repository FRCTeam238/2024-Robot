// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import static frc.robot.Constants.FeederConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */
  public IntakeNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.feeder, Robot.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.feeder.rollerController(speed);
    Robot.intake.setSpeed(intakeSpeed);
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.feeder.rollerController(0);
    Robot.intake.setSpeed(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return Robot.feeder.sensor();
  }
}
