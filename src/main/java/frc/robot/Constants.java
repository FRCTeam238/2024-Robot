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
    public static double intakeSpeed = 1;
    public static double ejectSpeed = -1;
    public static double ejectTime = 2; 
    public static double spinupDuration = 1; // TODO: Change to a better number
    public static double stallVelocity = 0; // TODO: Change to a better number
    public static double reverseDuration = 1; // TODO: Change to a better number
  }

  public class ShooterConstants {

    public static int leftMotorId = 8;
    public static int rightMotorId = 7;
    public static double kP = .5;
    public static double kI;
    public static double kD;
    public static double kFF = 0.116504854368932;
    public static double shooterTolerance = 30. / 60.; // 30 RPM, convert to RPS

    public static double subwooferLeft =
        4000. / 60; // convert to RPS | value * 1.33 = output shaft rps
    public static double subwooferRight = 3700. / 60.; // convert to RPS
    public static double podiumLeft = 4200. / 60.;
    public static double podiumRight = 3700. / 60.;

    public static double ampSpeed = 2000. / 60;
    public static double trapSpeed = 2000. / 60;
    public static double shotTime = .6;
    public static double arbitraryShotTime = .75; //specifically for fixed shot auto stuff because of potential extra spin up time
    public static double speedDifference = 20; //Percent difference in speed for targeted shots

    public static InterpolatingDoubleTreeMap rpmTree = new InterpolatingDoubleTreeMap();

    static {
      // DISCLAIMER: numbers are subject to change
      rpmTree.put(1.2065, 1432.39 / 60);
      rpmTree.put(2.66065, 1671.13 / 60);
      rpmTree.put(4.1021, 2053.1 / 60);
      rpmTree.put(5.2832, 2578.31 / 60);
      rpmTree.put(7.7216, 3485.49 / 60);
      rpmTree.put(10.1346, 4058.45 / 60);
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
    public static double kS = 0;
    public static double kG = 0;
    public static double kV = 2.3; // V*s/rad
    public static double kA = 0.1;

    public static double kP = 2;
    public static double kI;
    public static double kD = .1;

    public static int currentLimit = 40;
    public static int pivotID = 3;

    public static double maxJerk = 10000;
    public static double maxAccel = 15;
    public static double maxVelocity = 2.5; // Max is 3.89?
    public static double velocityTolerance = .05;
    public static final double velocityMaxError = 0.05;
    public static final double positionMaxError = 0.03;

    // Positions should all be in radians where 0 = horizontal
    public static final double intakePosition = Units.degreesToRadians(24.32);
    public static final double ampPosition = Units.degreesToRadians(-30);
    public static final double trapPosition = Units.degreesToRadians(30);
    public static final double climbPosition = 0;
    public static final double subwooferPosition = Units.degreesToRadians(50);
    public static final double podiumPosition = Units.degreesToRadians(30);
    public static final double feedOutPosition = Units.degreesToRadians(15);

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

    public static final double ejectRunTime = 4; // seconds
    public static final double ejectPercent = -0.6;
    public static final double feedSpeed = 1;
  }

  public class ElevatorConstants {
    public enum ElevatorDirection {
      UP,
      DOWN
    }

    public static final int leaderId = 2;
    public static final int followerId = 12;
    public static final double kP = 0.045;
    public static final double kI = 0;
    public static final double kD = 0.015; // 0.005 or 0.01
    public static final int currentLimit = 40;
    public static final float softForwardLimit = 25;
    public static final float softReverseLimit = 0.25f;
    public static final double kS = 0.1;
    public static final double kG = 0.37;
    public static final double kV = .31; // V*s/in
    public static final double kA = 0.0012; // V*s/in^2

    public static final double maxElevatorJerk = 5000;
    public static final double maxAccel = 200; // in/s^2 Max = 450?
    public static final double maxVelocity = 30; // in/s Max = 36?
    public static final double velocityTolerance = 0.5;
    public static final double velocityMaxError = 0.3;
    public static final double positionMaxError = 0.5;

    // All positions should be in inches
    public static final double intakePosition = 0.1;
    public static final double ampPosition = 24;
    public static final double trapPosition = 0;
    public static final double climbPosition = 31;
    public static final double subwooferPosition = 7;
    public static final double podiumPosition = 5;//TODO: change to real numbers
    public static final double feedOutPosition = 0;

    public static final double gearing = 12; // 12:1 gear ratio
    public static final double inchesPerRev = 1.5 * Math.PI; // 1.5" diameter pulley
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

    public static final double kP = 10;
    public static final double kI = 0;
    public static final double kD = 0.01;

    public static final double kPAngular = 4;
    public static final double kIAngular = 0;
    public static final double kDAngular = 0;

    public static final double llkP = .2;
    public static final double llkI = 0;
    public static final double llkD = 0;

    public static final double maxVelocityMetersPerSec = 4.3;
    public static final double maxAccelerationMetersPerSec2 = 100; // TODO: make this a real number

    public static final double kTrackWidth = Units.inchesToMeters(20.75);
    public static final double kWheelBase = Units.inchesToMeters(20.75);
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d((kWheelBase / 2) - .0508, kTrackWidth / 2),
            new Translation2d((kWheelBase / 2) - .0508, -kTrackWidth / 2),
            new Translation2d((-kWheelBase / 2) - .0508, kTrackWidth / 2),
            new Translation2d((-kWheelBase / 2) - .0508, -kTrackWidth / 2));

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
    SUBWOOFER,
    PODIUM,
    FEEDOUT,
    DASHBOARD,
    AUTO
  }

  public class VisionConstants {
    // TODO: change these to not zeroes please
    public static Transform3d frontCameraTransform =
        new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));

    public static Transform3d shooterCameraTransform =
        new Transform3d(-.230, .282, -.62, new Rotation3d(0, Units.degreesToRadians(22), 0));

    public static boolean updatesInTeleop = true;
    public static boolean updatesInAuto = true;
    public static double poseEstimateDistanceTolerance = 10000000;
    /**
     * {@summary units in radians}
    */
    public static double poseEstimateRotTolerance = 1;
  }

  
}
