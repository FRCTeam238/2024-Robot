// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.IntakeConstants.intakeSpeed;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class ClearNote extends Command {
  /** Creates a new ClearNode. */
  public ClearNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.intake.setSpeed(-intakeSpeed);
    OI.operatorController.getHID().setRumble(RumbleType.kBothRumble, 1);
    Robot.intake.setCommand("ClearNote");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.intake.setSpeed(0);
    OI.operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
    Robot.intake.setCommand("None");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
