package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants.ElevatorDirection;
import frc.robot.commands.*;
import monologue.Annotations.Log;
import monologue.Logged;

/** OI */
public class OI implements Logged {

  @Log.NT public static CommandJoystick leftJoystick = new CommandJoystick(2);
  @Log.NT public static CommandJoystick rightJoystick = new CommandJoystick(1);

  public static CommandXboxController driverController = new CommandXboxController(3);
  @Log.NT public static CommandXboxController operatorController = new CommandXboxController(0);

  static DriveType driveType = DriveType.XBOX;
  private static SendableChooser<DriveType> driveTypeChooser;

  public OI() {
    Robot.drivetrain.setDefaultCommand(new Drive());
    leftJoystick.button(20).onTrue(Robot.drivetrain.zeroHeadingCommand());
    rightJoystick.button(20).onTrue(Robot.drivetrain.zeroHeadingCommand());

    driveTypeChooser = new SendableChooser<>();
    driveTypeChooser.addOption("XBOX", DriveType.XBOX);
    driveTypeChooser.setDefaultOption("JOYSTICK", DriveType.JOYSTICK);
    SmartDashboard.putData(driveTypeChooser);




    operatorController.axisGreaterThan(5, 0.1).whileTrue(new ManualElevator(ElevatorDirection.UP)); //RightY
    operatorController.axisLessThan(5, -0.1).whileTrue(new ManualElevator(ElevatorDirection.DOWN));           //Right Y
    operatorController.axisGreaterThan(1, 0.1).whileTrue(new ManualPivot()); //Left Y
    operatorController.axisLessThan(1, -0.1).whileTrue(new ManualPivot());//Left Y
    operatorController.leftTrigger(0.27).whileTrue(new IntakeNote());
    operatorController.rightTrigger(0.27).whileTrue(new ScoreNote());

    leftJoystick.button(4).whileTrue(new AimDT());
    rightJoystick.button(4).whileTrue(new AimDT());

    leftJoystick.button(1).whileTrue(new LaunchGroup());
    rightJoystick.button(1).whileTrue(new LaunchGroup());
//    operatorController.rightBumper().whileTrue(new ScoreNote());

    operatorController.a().onTrue(new IntakePosition());
    operatorController.x().onTrue(new SubwooferPosition());
    operatorController.b().onTrue(new SubwooferPosition());
    operatorController.y().onTrue(new AmpPosition());
    operatorController.povDown().whileTrue(new IntakeNote());
    operatorController.povUp().whileTrue(new ClearNote());
    operatorController.povRight().whileTrue(new EjectNote());
  }

  private static boolean getSlowmode() {
    return leftJoystick.getHID().getRawButton(3) || rightJoystick.getHID().getRawButton(3);

  }

  @Log.NT
  public static double getRawRightX() {
    return rightJoystick.getX();
  }

  @Log.NT
  public static double getRawLeftY() {
    return leftJoystick.getY();
  }

  @Log.NT
  public static double getRawLeftX() {
    return rightJoystick.getX();
  }

  @Log.NT
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
