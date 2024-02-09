package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;

import static frc.robot.Constants.DriveConstants.*;

/** for swerve */
public class Drivetrain extends SubsystemBase {

  SwerveModule frontLeft =
      new SwerveModule(frontLeftDriveCANId, frontLeftTurnCANId);
  SwerveModule frontRight =
      new SwerveModule(frontRightDriveCANId, frontRightTurnCANId);
  SwerveModule backLeft =
      new SwerveModule(backLeftDriveCANId, backLeftTurnCANId);
  SwerveModule backRight =
      new SwerveModule(backRightDriveCANId, backRightTurnCANId);

  //SwerveDriveOdometry odometry;
  SwerveDrivePoseEstimator odometry;

  AHRS gyro;

  StructPublisher<Pose2d> publisher;

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
            }, new Pose2d());
    initializeTelemetry();
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
    updateTelemetry();
  }

  public void initializeTelemetry() {
    publisher =
        NetworkTableInstance.getDefault().getStructTopic("/RobotPose", Pose2d.struct).publish();
    SmartDashboard.putNumber("swerve/moduleCount", 4);
    SmartDashboard.putNumberArray(
        "swerve/wheelLocations",
        new double[] {
          new Translation2d(kWheelBase / 2, kTrackWidth / 2).getX(),
          new Translation2d(kWheelBase / 2, kTrackWidth / 2).getY(),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2).getX(),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2).getY(),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2).getX(),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2).getY(),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2).getX(),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2).getY(),
        });
    SmartDashboard.putString("swerve/rotationUnit", "radians");
    SmartDashboard.putNumber("swerve/sizeLeftRight", kTrackWidth);
    SmartDashboard.putNumber("swerve/sizeFrontBack", kWheelBase);
    SmartDashboard.putString("swerve/forwardDirection", "up");
    SmartDashboard.putNumber(
        "swerve/maxAngularVelocity", maxAngularVelocityRadsPerSec);
    SmartDashboard.putNumber("swerve/maxSpeed", maxVelocityMetersPerSec);
  }

  public void updateTelemetry() {
    publisher.set(getPose());
    SmartDashboard.putNumberArray(
        "swerve/measuredStates",
        new double[] {
          frontLeft.getState().angle.getRadians(), frontLeft.getState().speedMetersPerSecond,
          frontRight.getState().angle.getRadians(), frontRight.getState().speedMetersPerSecond,
          backLeft.getState().angle.getRadians(), backLeft.getState().speedMetersPerSecond,
          backRight.getState().angle.getRadians(), backRight.getState().speedMetersPerSecond,
        });
    SmartDashboard.putNumberArray(
        "swerve/desiredStates",
        new double[] {
          frontLeft.getDesiredState().angle.getRadians(),
              frontLeft.getDesiredState().speedMetersPerSecond,
          frontRight.getDesiredState().angle.getRadians(),
              frontRight.getDesiredState().speedMetersPerSecond,
          backLeft.getDesiredState().angle.getRadians(),
              backLeft.getDesiredState().speedMetersPerSecond,
          backRight.getDesiredState().angle.getRadians(),
              backRight.getDesiredState().speedMetersPerSecond,
        });
    SmartDashboard.putNumber("swerve/robotRotation", gyro.getRotation2d().getRadians());
    ChassisSpeeds measuredChassisSpeeds =
        kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    ChassisSpeeds desiredChassisSpeeds =
        kDriveKinematics.toChassisSpeeds(
            frontLeft.getDesiredState(),
            frontRight.getDesiredState(),
            backLeft.getDesiredState(),
            backRight.getDesiredState());

    SmartDashboard.putNumberArray(
        "swerve/measuredChassisSpeeds",
        new double[] {
          measuredChassisSpeeds.vyMetersPerSecond,
          measuredChassisSpeeds.vxMetersPerSecond,
          measuredChassisSpeeds.omegaRadiansPerSecond
        });
    SmartDashboard.putNumberArray(
        "swerve/desiredChassisSpeeds",
        new double[] {
          desiredChassisSpeeds.vyMetersPerSecond,
          desiredChassisSpeeds.vxMetersPerSecond,
          desiredChassisSpeeds.omegaRadiansPerSecond
        });
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void updatePoseEstimate(EstimatedRobotPose estimate)
  {
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

  public void drive(double xSpeed, double ySpeed, double rot) {

    var swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

    setModuleStates(swerveModuleStates);
  }

  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates);
  }

  public void lockWheels() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
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

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState(),
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
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return gyro.getRate();
  }  
}
