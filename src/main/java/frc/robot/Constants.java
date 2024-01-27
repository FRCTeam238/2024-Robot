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
        
    }

    public class OperatorConstants {
        public static double driverJoystickDeadzone = .1;
        public static double xboxControllerDeadzone = .075; //TODO: find good deadzone values for the xbox controllers
        public enum DriveType {
            JOYSTICK,
            XBOX
        }
    }
}