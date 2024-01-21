package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

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

        double leftJoyX = Robot.oi.leftJoystick.getX();

        if (Math.abs(leftJoyX) <= .1){
            leftJoyX = 0;
        }
        double rightJoyX = Robot.oi.rightJoystick.getX();
        if (Math.abs(rightJoyX) <= .1){
            rightJoyX = 0;
        }
        double leftJoyY = Robot.oi.leftJoystick.getY();
        if (Math.abs(leftJoyY) <= .1){
            leftJoyY = 0;
        }
        var maxSpeed = Drivetrain.DriveConstants.maxVelocityMetersPerSec;
        var maxAng = Drivetrain.DriveConstants.maxAngularVelocityRadsPerSec;

        drivetrain.drive(-leftJoyY*maxSpeed, -leftJoyX*maxSpeed, -rightJoyX * maxAng);
        
    }
    
}
