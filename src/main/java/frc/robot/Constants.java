package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
  public class IntakeConstants {
    public static int kID = 13;
    public static double intakeSpeed = 0;
    public static double ejectTime = 2;
  }

  public class ShooterConstants {

    public static int leftMotorId = 8;
    public static int rightMotorId = 7;
    public static double kP;
    public static double kI;
    public static double kD;
    public static double kFF;
    public static double shooterTolerance = 0.3; // TODO: see  if another value is better.


    public static InterpolatingDoubleTreeMap rpmTree = new InterpolatingDoubleTreeMap();

    static {
      //DISCLAIMER: numbers are subject to change
      rpmTree.put(1.2065, 1432.39);
      rpmTree.put(2.66065,1671.13);
      rpmTree.put(4.1021, 2053.1);
      rpmTree.put(5.2832, 2578.31);
      rpmTree.put(7.7216, 3485.49);
      rpmTree.put(10.1346, 4058.45);
    }

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
    public static double voltageMax = 12;
    public static double kS = .1;
    public static double kG = .17;
    public static double kV = 2.99; //V*s/rad

    public static double kP = 5; //estimating max error tolerance should result in an output of at least .05
    public static double kI;
    public static double kD;

    public static int currentLimit = 40;
    public static int motor1 = 2;

    public static double maxJerk = 10000;
    public static double maxAccel = 10;
    public static double maxVelocity = 1; //Max is 3.89?
    public static double velocityTolerance = .05;
    public static final double velocityMaxError = 0.005; //~.25 degrees/s
    public static final double positionMaxError = 0.009; //.5 degrees

    //Positions should all be in radians where 0 = horizontal
    public static final double intakePosition = 0;
    public static final double ampPosition = 0;
    public static final double trapPosition = 0;
    public static final double climbPosition = 0;
    public static final double speakerPosition = 1.05; //~60 degrees

    public static InterpolatingDoubleTreeMap pivotAngles = new InterpolatingDoubleTreeMap();

    static {
      pivotAngles.put(1.2065, Units.degreesToRadians(60.));
      pivotAngles.put(2.66065, Units.degreesToRadians(43.));
      pivotAngles.put(4.1021, Units.degreesToRadians(32.));
      pivotAngles.put(5.2832, Units.degreesToRadians(25.));
      pivotAngles.put(7.7216, Units.degreesToRadians(18.));
      pivotAngles.put(10.1346, Units.degreesToRadians(15.));
    }

  }

  public class FeederConstants {
    public static final int feederId = 6;
    public static final int sensorId = 0;

    public static final double ejectRunTime = 4;//seconds
    public static final double ejectPercent = -0.5;
    public static final double feedSpeed = 1;
  }

  public class ElevatorConstants {
    public static final int leaderId = 3;
    public static final int followerId = 12;
    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int currentLimit = 40;
    public static final double kS = 0.1;
    public static final double kG = 0.42;
    public static final double kV = .31; //V*s/in
    public static final double kA = 0.0016; //V*s/in^2

    public static final double maxElevatorJerk = 10000;
    public static final double maxAccel = 50; //in/s^2 Max = 450?
    public static final double maxVelocity = 10;//in/s Max = 36?
    public static final double velocityTolerance = 0.5;
    public static final double velocityMaxError = 0.2;
    public static final double positionMaxError = 0.5;

    //All positions should be in inches
    public static final double intakePosition = 0;
    public static final double ampPosition = 0;
    public static final double trapPosition = 0;
    public static final double climbPosition = 0;
    public static final double speakerPosition = 6;

    public final static double gearing = 12; // 4:1 gear ratio
    public final static double inchesPerRev = 1.5 * Math.PI; // 1.5" diameter pulley
  }

  public class DriveConstants {

    public static final boolean fieldRelative = true;

    public static final int frontRightDriveCANId = 14;
    public static final int frontLeftDriveCANId = 15;
    public static final int backRightDriveCANId = 1;
    public static final int backLeftDriveCANId = 0;

    public static final int frontRightTurnCANId = 10;
    public static final int backRightTurnCANId = 5;
    public static final int backLeftTurnCANId = 4;
    public static final int frontLeftTurnCANId = 11;

    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kPAngular = 1;
    public static final double kIAngular = 0;
    public static final double kDAngular = 0;

    public static final double maxVelocityMetersPerSec = 4.86;
    public static final double maxAccelerationMetersPerSec2 = 100; // TODO: make this a real number

    public static final double kTrackWidth = Units.inchesToMeters(20.75);
    public static final double kWheelBase = Units.inchesToMeters(20.75);
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d((kWheelBase / 2) - 2, kTrackWidth / 2),
            new Translation2d((kWheelBase / 2) - 2, -kTrackWidth / 2),
            new Translation2d((-kWheelBase / 2) - 2, kTrackWidth / 2),
            new Translation2d((-kWheelBase / 2) - 2, -kTrackWidth / 2));

    public static double maxAngularVelocityRadsPerSec = 2 * Math.PI;

    public static final double turnTolerance = 0;
    public static final double velocityTolerance = 0.1;
    public static final double positionTolerance = 0.05;
    public static final double xandyvelocityTolerance = 0.05;

    public static final double stageOffset = Units.inchesToMeters(8);
  }

  public enum RobotState {
    AMP,
    TRAP,
    INTAKE,
    TARGET,
    SPEAKER
  }

  public class VisionConstants {
    //TODO: change these to not zeroes please
    public static Transform3d frontCameraTransform = new Transform3d(0, 0, 0, new Rotation3d(0,0,0));
    //TODO: change these to not zeroes please
    public static Transform3d backCameraTransform = new Transform3d(0, 0, 0, new Rotation3d(0,0,0));
  }
}

