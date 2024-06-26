// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.IntakeConstants.ejectTime;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.frc238.lib.autonomous.AutonomousModeAnnotation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@AutonomousModeAnnotation(parameterNames = {})
public class IntakeGroupAuto extends ParallelCommandGroup {
  /** Creates a new Intake. */
  public IntakeGroupAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new IntakePosition(), new IntakeNote());
  }
}
