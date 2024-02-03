package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    return false;
  }

  public static Pose2d speakerLocation() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      switch (DriverStation.getAlliance().get()) {
        case Red -> {
          return new Pose2d(16.541, 5.548, new Rotation2d());
        }
        case Blue -> {
          return new Pose2d(0, 5.548, new Rotation2d());
        }
      }
    }
    return new Pose2d(0, 5.548, new Rotation2d());
  }

}
