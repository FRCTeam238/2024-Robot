package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import org.frc238.lib.autonomous.AutonomousModeAnnotation;

import java.util.Map;

@AutonomousModeAnnotation(parameterNames = {})
public class ScoreNote extends SequentialCommandGroup {

    public ScoreNote() {
        // we will check if we are in either ampPosition or literally anything else
        addCommands(new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(Constants.RobotState.INTAKE, new LaunchSubwooferGroup()),
                        Map.entry(Constants.RobotState.TARGET, new LaunchGroup()),
                        Map.entry(Constants.RobotState.SUBWOOFER, new LaunchSubwooferGroup()),
                        Map.entry(Constants.RobotState.AMP, new ScoreAmpGroup()),
                        Map.entry(Constants.RobotState.TRAP, new ScoreTrapGroup())
                ),
                Robot::getState
        ));
    }

}
