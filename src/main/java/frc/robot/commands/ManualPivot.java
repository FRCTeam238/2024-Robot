// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorDirection;
import frc.robot.OpInterface;
import frc.robot.Robot;
import frc.robot.subsystems.Pivot;

public class ManualPivot extends Command {
  Pivot pivot = Robot.pivot;
  ElevatorDirection direction;

  /** Creates a new ManualElevator. */
  public ManualPivot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.setCommand("ManualPivot");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.setSpeed(OpInterface.operatorController.getRightY() * .25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.holdPosition();
    pivot.setCommand("None");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
