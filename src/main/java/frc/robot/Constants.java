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
        public static double shooterTolerance = 0.3;//TODO: see  if another value is better.
    }

    public class OperatorConstants {
        public static double driverJoystickDeadzone = .1;
        public static double xboxControllerDeadzone = .075; //TODO: find good deadzone values for the xbox controllers
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

        public static double currentLimit;
        public static int motor1;
    }
}
