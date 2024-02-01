package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;
import org.frc238.lib.autonomous.AutonomousModeAnnotation;

/** TrajectoryDriveCommand */
@AutonomousModeAnnotation(parameterNames = {"TrajectoryName", "resetPosition", "maxVelocity"})
public class TrajectoryDriveCommand extends SequentialCommandGroup {

  Drivetrain drivetrain = Robot.drivetrain;

  /**
   * takes the name of a path made in the software of choice, loads it, and makes a
   * SwerveControllerCommand
   *
   * @param pathName name of the trajectory as a string
   * @param resetPosition set to true to set the robot's position to the start of the trajectory -
   *     helpful if this is the first trajectory you want to run
   * @param maxVelocity in meters per second - if set to zero, the default maximum velocity of the
   *     drivetrain is used
   */
  public TrajectoryDriveCommand(String pathName, boolean resetPosition, double maxVelocity) {
    ChoreoTrajectory trajectory = Choreo.getTrajectory(pathName);

    // reference to the method used to get the current pose
    Supplier<Pose2d> pose = drivetrain::getPose;
    PIDController xController =
        new PIDController(
            Drivetrain.DriveConstants.kP,
            Drivetrain.DriveConstants.kI,
            Drivetrain.DriveConstants.kD);
    PIDController yController =
        new PIDController(
            Drivetrain.DriveConstants.kP,
            Drivetrain.DriveConstants.kI,
            Drivetrain.DriveConstants.kD);
    PIDController rotationController =
        new PIDController(
            Drivetrain.DriveConstants.kPAngular,
            Drivetrain.DriveConstants.kIAngular,
            Drivetrain.DriveConstants.kDAngular);

    // creates a command that sets the position of the robot to the starting point of the trajectory
    Command resetPos =
        drivetrain.runOnce(() -> drivetrain.resetOdometry(trajectory.getInitialPose()));
    resetPos.addRequirements(drivetrain);
    if (resetPosition) {
      addCommands(resetPos);
    }

    Command swerveCommand =
        Choreo.choreoSwerveCommand(
            trajectory,
            pose,
            xController,
            yController,
            rotationController,
            drivetrain::driveWithChassisSpeeds,
            Utils::isPathReversed);

    addCommands(swerveCommand);
  }
}
