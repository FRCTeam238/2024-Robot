// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Drive;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
    Robot.drivetrain.setDefaultCommand(new Drive());
  }

  private void configureBindings() {
    OI.driverController.start().onTrue(Robot.drivetrain.zeroHeadingCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
