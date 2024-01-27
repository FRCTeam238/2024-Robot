package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.subsystems.Drivetrain.DriveConstants.*;

/**
 * Drive
 */
public class Drive extends Command {

    Drivetrain drivetrain = Robot.drivetrain;

    public Drive() {
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        
        
    }

    @Override
    public void execute() {
        // if the |joystick x axis| is less than 10%, then set leftJoyX to 0

        double[] joyValues = OI.getSwerveJoystickValues();

        drivetrain.drive(joyValues[0]*maxVelocityMetersPerSec, joyValues[1]*maxVelocityMetersPerSec, joyValues[2]* maxAngularVelocityRadsPerSec);
        
    }
    
}
