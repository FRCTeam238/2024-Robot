package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
  public class IntakeConstants {
    public static int kID = 0;
    public static double intakeSpeed = 0;
  }

  public class ShooterConstants {
    public static double kP;
    public static double kI;
    public static double kD;
    public static double kFF;
    public static double shooterTolerance = 0.3; // TODO: see  if another value is better.
  }

  public class OperatorConstants {
    public static double driverJoystickDeadzone = .1;
    public static double xboxControllerDeadzone =
        .075; // TODO: find good deadzone values for the xbox controllers

    public enum DriveType {
      JOYSTICK,
      XBOX
    }
  }

  public class PivotConstants {
    public static double voltageMax;
    public static double kS;
    public static double kG;
    public static double kV;

    public static double kP;
    public static double kI;
    public static double kD;

    public static int currentLimit;
    public static int motor1;

    public static double maxJerk = 0;
    public static double maxAccel = 0;
    public static double maxVelocity = 0;
    public static double velocityTolerance = 0;
    public static final double pivotTolerance = 0.1;
  }

  public class FeederConstants {
    public static final int feederId = 0;
    public static final int sensorId = 0;

    public static final double ejectRunTime = 4;//seconds
    public static final double ejectPercent = -0.5;
    public static final double speed = 0;
  }

  public class ElevatorConstants {
    public static final int leaderId = 0;
    public static final int followerId = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int currentLimit = 0;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double maxElevatorJerk = 0;
    public static final double maxAccel = 0;
    public static final double maxVel = 0;
    public static final double velocityTolerance = 0;

    public static final double velocityMaxError = 0;
    public static final double positionMaxError = 0;

    public static final double elevatorTolerance = .1;
  }

  public class DriveConstants {

    public static final boolean fieldRelative = true;

    public static final int frontRightDriveCANId = 0;
    public static final int backRightDriveCANId = 2;
    public static final int backLeftDriveCANId = 4;
    public static final int frontLeftDriveCANId = 6;

    public static final int frontRightTurnCANId = 1;
    public static final int backRightTurnCANId = 3;
    public static final int backLeftTurnCANId = 5;
    public static final int frontLeftTurnCANId = 7;

    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kPAngular = 1;
    public static final double kIAngular = 0;
    public static final double kDAngular = 0;

    public static final double maxVelocityMetersPerSec = 4.86;
    public static final double maxAccelerationMetersPerSec2 = 100; // TODO: make this a real number

    public static final double kTrackWidth = Units.inchesToMeters(20);
    public static final double kWheelBase = Units.inchesToMeters(20);
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static double maxAngularVelocityRadsPerSec = 2 * Math.PI;

    public static final double turnTolerance = 0;
    public static final double velocityTolerance = 0.1;
    public static final double positionTolerance = 0.05;
    public static final double xandyvelocityTolerance = 0.05;
  }
}
