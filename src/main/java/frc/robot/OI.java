package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive;

/** OI */
public class OI {

  public static Joystick leftJoystick = new Joystick(2);
  public static Joystick rightJoystick = new Joystick(1);

  public static CommandXboxController driverController = new CommandXboxController(3);
  public static CommandXboxController operatorController = new CommandXboxController(0);

  static DriveType driveType = DriveType.XBOX;
  private static SendableChooser<DriveType> driveTypeChooser;

  public OI() {
    Robot.drivetrain.setDefaultCommand(new Drive());
    OI.driverController.start().onTrue(Robot.drivetrain.zeroHeadingCommand());
    driveTypeChooser = new SendableChooser<>();
    driveTypeChooser.addOption("XBOX", DriveType.XBOX);
    driveTypeChooser.setDefaultOption("JOYSTICK", DriveType.JOYSTICK);
    SmartDashboard.putData(driveTypeChooser);
  }

  private static boolean getSlowmode() {
    return driverController.getHID().getAButton() || leftJoystick.getRawButton(2) || rightJoystick.getRawButton(2);
  }

  public static double[] getSwerveJoystickValues() {
    double slowmodePercent =  getSlowmode() ? .75 : 1;
    
    switch (getDriveType()) {
      case JOYSTICK -> {
        return new double[] {
          // applyDeadband will do the absolute value stuff for us and make the zero point start at
          // the deadzone edge
          Math.pow(-MathUtil.applyDeadband(leftJoystick.getY(), driverJoystickDeadzone), 1) * slowmodePercent,
          Math.pow(-MathUtil.applyDeadband(leftJoystick.getX(), driverJoystickDeadzone), 1) * slowmodePercent,
          Math.pow(-MathUtil.applyDeadband(rightJoystick.getX(), driverJoystickDeadzone), 1),
        };
      }
      case XBOX -> {
        return new double[] {
          // applyDeadband will do the absolute value stuff for us and make the zero point start at
          // the deadzone edge
          Math.pow(-MathUtil.applyDeadband(driverController.getLeftY(), xboxControllerDeadzone), 5) * slowmodePercent,
          Math.pow(-MathUtil.applyDeadband(driverController.getLeftX(), xboxControllerDeadzone), 5) * slowmodePercent,
          Math.pow(-MathUtil.applyDeadband(driverController.getRightX(), xboxControllerDeadzone), 3),
        };
      }
      default -> throw new IllegalStateException("Unexpected value: " + getDriveType());
    }
  }
  public static DriveType getDriveType() {
      driveType = driveTypeChooser.getSelected();
      return driveType;
  }
}
