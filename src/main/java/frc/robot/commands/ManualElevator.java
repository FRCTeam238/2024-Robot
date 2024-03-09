// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorDirection;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class ManualElevator extends Command {
  Elevator elevator = Robot.elevator;
  ElevatorDirection direction;

  /** Creates a new ManualElevator. */
  public ManualElevator(ElevatorConstants.ElevatorDirection direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    this.direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setCommand("Manual");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (direction) {
      case DOWN:
        elevator.setSpeed(OI.operatorController.getRightY());
        break;
      case UP:
        elevator.setSpeed(OI.operatorController.getRightY());
        break;
      default:
        elevator.setSpeed(0);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.holdPosition();
    elevator.setCommand("None");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
