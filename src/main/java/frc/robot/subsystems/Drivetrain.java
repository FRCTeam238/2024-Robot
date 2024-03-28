package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.choreo.lib.ChoreoTrajectoryState;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;

/** for swerve */
public class Drivetrain extends SubsystemBase implements Logged {

  SwerveModule frontLeft = new SwerveModule(frontLeftDriveCANId, frontLeftTurnCANId);
  SwerveModule frontRight = new SwerveModule(frontRightDriveCANId, frontRightTurnCANId);
  SwerveModule backLeft = new SwerveModule(backLeftDriveCANId, backLeftTurnCANId);
  SwerveModule backRight = new SwerveModule(backRightDriveCANId, backRightTurnCANId);

  SwerveDrivePoseEstimator odometry;
  AHRS gyro;
  PIDController x, y, theta;
  @Log String command;

  public Drivetrain() {
    gyro = new AHRS(Port.kMXP);
    odometry =
        new SwerveDrivePoseEstimator(
            kDriveKinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              backLeft.getPosition(),
              backRight.getPosition()
            },
            new Pose2d());

    x = new PIDController(kP, kI, kD);
    y = new PIDController(kP, kI, kD);
    theta = new PIDController(kPAngular, kIAngular, kDAngular);
    theta.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }

  @Log.NT
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void setCommand(String name) {
    command = name;
  }

  public void updatePoseEstimate(EstimatedRobotPose estimate) {
    odometry.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds);
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        },
        pose);
  }

  public Rotation2d getFieldRelativeOffset() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        return odometry.getEstimatedPosition().getRotation();
      } else {
        return odometry
            .getEstimatedPosition()
            .getRotation()
            .minus(Rotation2d.fromDegrees(180)); // DO NOT USE IF WE DONT RUN A PATH
      }
    } else {
      return odometry.getEstimatedPosition().getRotation();
    }
  }

  public void drive(double xSpeed, double ySpeed, double rot) {

    var swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, getFieldRelativeOffset())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                .02));
    setModuleStates(swerveModuleStates);
  }

  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates);
  }

  public void lockWheels() {
    SwerveModuleState wheelLock[] = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    };
    setModuleStates(wheelLock);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocityMetersPerSec);

    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  @Log
  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState(),
    };
  }

  @Log
  public SwerveModuleState[] getDesiredStates() {
    return new SwerveModuleState[] {
            frontLeft.getDesiredState(), frontRight.getDesiredState(), backLeft.getDesiredState(), backRight.getDesiredState()
    };
  }

  public SwerveModulePosition[] getSwerveModulePosition() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition(),
    };
  }

  public void zeroHeading() {
    gyro.reset();
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        gyro.setAngleAdjustment(180);
      } else {
        gyro.setAngleAdjustment(0);
      }
    }
  }

  public Command zeroHeadingCommand() {
    return Commands.runOnce(
        () -> {
          this.zeroHeading();
        });
  }

  @Log.NT
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return gyro.getRate();
  }

  public ChassisSpeeds choreoController(Pose2d currentPose, ChoreoTrajectoryState referenceState) {
    log("TrajPose", referenceState.getPose());
    double xFF = referenceState.velocityX;
    double yFF = referenceState.velocityY;
    double rotationFF = referenceState.angularVelocity;

    double xFeedback = x.calculate(currentPose.getX(), referenceState.x);
    double yFeedback = y.calculate(currentPose.getY(), referenceState.y);
    double rotationFeedback =
        theta.calculate(currentPose.getRotation().getRadians(), referenceState.heading);
    log("CurrentRotation", currentPose.getRotation().getRadians());
    log("PoseRotation", referenceState.getPose().getRotation().getRadians());
    ChassisSpeeds retval = ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, currentPose.getRotation());
    log("ChassisSpeeds", retval);
    return retval;
  }
}
