// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Utils;

import org.frc238.lib.autonomous.AutonomousModeAnnotation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@AutonomousModeAnnotation(parameterNames = {})
public class TriggeredTargetSubwoofer extends SequentialCommandGroup {
  /** Creates a new AmpPosition. */
  public TriggeredTargetSubwoofer() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.waitUntil(()->Utils.getSpeakerDistance() < 3.25),
        new TargetSubwoofer());
  }
}
