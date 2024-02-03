package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import static frc.robot.Constants.FeederConstants.*;

public class EjectNote extends Command {
    private final Feeder feeder;

    public EjectNote(Feeder feeder) {
        this.feeder = feeder;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.feeder);

    }

    @Override
    public void execute() {
        feeder.rollerController(ejectPercent);

    }

    @Override
    public void end(boolean interrupted) {
        feeder.rollerController(0);

    }
}
