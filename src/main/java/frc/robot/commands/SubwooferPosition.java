// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.MotionProfile.State;
import frc.robot.Robot;
import org.frc238.lib.autonomous.AutonomousModeAnnotation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@AutonomousModeAnnotation(parameterNames = {})
public class SubwooferPosition extends ParallelCommandGroup {
  /** Creates a new AmpPosition. */
  public SubwooferPosition() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new RunCommand(() -> Robot.state = Constants.RobotState.SUBWOOFER),
        new ElevatorProfile(new State(ElevatorConstants.subwooferPosition), "SubwooferPosition"),
        new PivotProfile(new State(PivotConstants.subwooferPosition), "SubwooferPosition"));
  }
}
