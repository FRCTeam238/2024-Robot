
/* (C)2023 */
package frc.robot.commands;

import org.frc238.lib.autonomous.AutonomousModeAnnotation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


@AutonomousModeAnnotation(parameterNames = {"delay"})
public class Delay extends CommandBase {
    private double m_delay;
    private double m_startTime;

    public Delay(double delay) {
        m_delay = delay;
        
    }

    @Override
    public void initialize() {
        m_startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - m_startTime >= m_delay;
    }
}
