package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.subsystems.Drivetrain;
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

    // creates a command that sets the position of the robot to the starting point of the trajectory
    Command resetPos =
        drivetrain.runOnce(
            () ->
                drivetrain.resetOdometry(
                    DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                        ? trajectory.flipped().getInitialPose()
                        : trajectory.getInitialPose()));
    resetPos.addRequirements(drivetrain);
    if (resetPosition) {
      addCommands(resetPos);
    }

    addCommands(drivetrain.runOnce(() -> drivetrain.setCommand("Traj-" + pathName)));

    Command swerveCommand =
        Choreo.choreoSwerveCommand(
            trajectory,
            drivetrain::getPose,
            drivetrain::choreoController,
            drivetrain::driveWithChassisSpeeds,
            Utils::isPathReversed);

    addCommands(swerveCommand);
    addCommands(drivetrain.runOnce(() -> drivetrain.setCommand("None")));
  }
}
