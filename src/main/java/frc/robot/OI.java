package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.Drive;

/**
 * OI
 */
public class OI {

    public Joystick leftJoystick = new Joystick(2);
    public Joystick rightJoystick = new Joystick(1);
    
    public OI() {
        Robot.drivetrain.setDefaultCommand(new Drive());
    }
}
