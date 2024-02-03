package frc.robot;

public class Constants {
  public class IntakeConstants {
    public static int kID = 0;
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

  }

  public class FeederConstants {
    public static final int feederId = 0;
    public static final int sensorId = 0;
    public static final double ejectRunTime = 4;//seconds
    public static final double ejectPercent = -0.5;
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

  }
}
