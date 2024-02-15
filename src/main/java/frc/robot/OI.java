package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive;

/** OI */
public class OI {

  public static Joystick leftJoystick = new Joystick(2);
  public static Joystick rightJoystick = new Joystick(1);

  public static CommandXboxController driverController = new CommandXboxController(3);
  public static CommandXboxController operatorController = new CommandXboxController(0);

  static DriveType driveType = DriveType.XBOX;

  public OI() {
    Robot.drivetrain.setDefaultCommand(new Drive());
  }

  public static double[] getSwerveJoystickValues() {
    switch (driveType) {
      case JOYSTICK -> {
        return new double[] {
          // applyDeadband will do the absolute value stuff for us and make the zero point start at
          // the deadzone edge
          -MathUtil.applyDeadband(leftJoystick.getY(), driverJoystickDeadzone),
          -MathUtil.applyDeadband(leftJoystick.getX(), driverJoystickDeadzone),
          -MathUtil.applyDeadband(rightJoystick.getX(), driverJoystickDeadzone),
        };
      }
      case XBOX -> {
        return new double[] {
          // applyDeadband will do the absolute value stuff for us and make the zero point start at
          // the deadzone edge
          -MathUtil.applyDeadband(driverController.getLeftY(), xboxControllerDeadzone),
          -MathUtil.applyDeadband(driverController.getLeftX(), xboxControllerDeadzone),
          -MathUtil.applyDeadband(driverController.getRightX(), xboxControllerDeadzone),
        };
      }
      default -> throw new IllegalStateException("Unexpected value: " + driveType);
    }
  }
}
