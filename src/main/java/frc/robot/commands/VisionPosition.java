// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.MotionProfile;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.MotionProfile.State;
import frc.robot.Robot;

import java.util.function.Supplier;

import org.frc238.lib.autonomous.AutonomousModeAnnotation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@AutonomousModeAnnotation(parameterNames = {"elevatorPosition", "pivotPosition"})
public class VisionPosition extends ParallelCommandGroup {

  Supplier<Double> pivotSupplier;
  /** Creates a new AmpPosition. */
  public VisionPosition(double elevatorPosition, Supplier<Double> pivotPosition) {
    // Add your commands in the addCommands() call, e.g.
    pivotSupplier = pivotPosition;
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> Robot.state = Constants.RobotState.AUTO),
        new ElevatorProfile(new State(elevatorPosition), "AutoPosition"),
        new VisionPivot(this::getNewState, "AutoPosition"));
  }


  MotionProfile.State getNewState() {
    return new State(pivotSupplier.get());
  }
}
