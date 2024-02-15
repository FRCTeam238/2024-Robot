package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.Map;

public class ReleaseNote extends SequentialCommandGroup {

    public ReleaseNote() {
        // we will check if we are in either ampPosition or literally anything else
        addCommands(new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(Constants.RobotState.INTAKE, new Launch()),
                        Map.entry(Constants.RobotState.TARGET, new Launch()),
                        Map.entry(Constants.RobotState.SPEAKER, new Launch()),
                        Map.entry(Constants.RobotState.AMP, new ScoreAmp()),
                        Map.entry(Constants.RobotState.TRAP, new ScoreTrap())
                ),
                Robot::getState
        ));
    }

}
