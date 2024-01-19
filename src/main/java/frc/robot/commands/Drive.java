package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

/**
 * Drive
 */
public class Drive extends CommandBase {

    Drivetrain drivetrain = Robot.drivetrain;

    public Drive() {
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        
        
    }

    @Override
    public void execute() {
        double leftJoyX = Robot.oi.leftJoystick.getX();
        double leftJoyY = Robot.oi.leftJoystick.getY();
        double rightJoyX = Robot.oi.rightJoystick.getX();

        drivetrain.drive(leftJoyX, leftJoyY, rightJoyX, true);
        
    }
    
}
