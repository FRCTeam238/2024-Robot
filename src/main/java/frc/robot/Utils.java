package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class Utils {
    public static boolean isPathReversed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            switch (DriverStation.getAlliance().get()) {
                case Red -> {
                    return true;
                }
                case Blue -> {
                    return false;
                }
            }
        }
        //TODO: ask kevin what we should do if this gets used before we are connected to DS and isPresent is called
        return false;
    }
}
