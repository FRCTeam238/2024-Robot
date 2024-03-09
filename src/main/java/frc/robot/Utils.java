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

  public static Pose2d ampLocation() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      switch (DriverStation.getAlliance().get()) {
        case Red -> {
          return new Pose2d(14.700, 7.738, new Rotation2d(Math.PI / 2));
        }
        case Blue -> {
          return new Pose2d(1.84, 7.738, new Rotation2d(Math.PI / 2));
        }
      }
    }
    return new Pose2d(1.84, 7.738, new Rotation2d(Math.PI / 2));
  }

  public static Pose2d stageLeft() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (DriverStation.getAlliance().get() != DriverStation.Alliance.Red) {
        return new Pose2d(
            4.641 - Constants.DriveConstants.stageOffset / 2,
            4.498 + Constants.DriveConstants.stageOffset * Math.sqrt(3) / 2,
            new Rotation2d(Math.PI * 5 / 3)); // or Blue
      }
      return new Pose2d(
          11.904 + Constants.DriveConstants.stageOffset / 2,
          3.713 - Constants.DriveConstants.stageOffset * Math.sqrt(3) / 2,
          new Rotation2d(Math.PI * 2 / 3));
    }
    return new Pose2d(
        11.904 + Constants.DriveConstants.stageOffset / 2,
        3.713 - Constants.DriveConstants.stageOffset * Math.sqrt(3) / 2,
        new Rotation2d(Math.PI * 2 / 3));
  }

  public static Pose2d stageRight() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (DriverStation.getAlliance().get() != DriverStation.Alliance.Red) {
        return new Pose2d(
            11.904 + Constants.DriveConstants.stageOffset / 2,
            4.498 + Constants.DriveConstants.stageOffset * Math.sqrt(3) / 2,
            new Rotation2d(Math.PI * 4 / 3)); // or Blue
      }
      return new Pose2d(
          4.641 + Constants.DriveConstants.stageOffset / 2,
          3.713 - Constants.DriveConstants.stageOffset * Math.sqrt(3) / 2,
          new Rotation2d(Math.PI / 3));
    }
    return new Pose2d(
        4.641 + Constants.DriveConstants.stageOffset / 2,
        3.713 - Constants.DriveConstants.stageOffset * Math.sqrt(3) / 2,
        new Rotation2d(Math.PI / 3));
  }

  public static Pose2d stageCenter() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (DriverStation.getAlliance().get() != DriverStation.Alliance.Red) {
        return new Pose2d(
            11.220 - Constants.DriveConstants.stageOffset, 4.105, new Rotation2d(0)); // or Blue
      }
      return new Pose2d(
          5.320 + Constants.DriveConstants.stageOffset, 4.105, new Rotation2d(Math.PI));
    }
    return new Pose2d(5.320 + Constants.DriveConstants.stageOffset, 4.105, new Rotation2d(Math.PI));
  }

  public static double getSpeakerDistance() {
    Pose2d currentPosition = Robot.drivetrain.getPose();
    Pose2d speakerLocation = Utils.speakerLocation();
    return currentPosition.getTranslation().getDistance(speakerLocation.getTranslation());
  }
}
