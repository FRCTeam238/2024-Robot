package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OpInterface;
import frc.robot.Robot;

import static frc.robot.Constants.DriveConstants.*;


public class SnapToAngle extends Command {
    private final double angle;
    private final PIDController pid;

    /**
     * snaps the robot to an angle relative to the field
     *
     * @param angle in degrees
     */
    public SnapToAngle(double angle) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(Robot.drivetrain);
        pid = new PIDController(kPAngular, kIAngular, kDAngular);
        pid.enableContinuousInput(-Math.PI, Math.PI);
        this.angle = angle;
    }

    @Override
    public void initialize() {
        Robot.drivetrain.setCommand("SnapToAngle");
    }

    @Override
    public void execute() {
        double[] joyValues = OpInterface.getSwerveJoystickValues();
        double robotAngle = Robot.drivetrain.getFieldRelativeOffset().getRadians();

        Robot.drivetrain.drive(
                joyValues[0] * maxVelocityMetersPerSec,
                joyValues[1] * maxVelocityMetersPerSec,
                pid.calculate(robotAngle, angle));
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
