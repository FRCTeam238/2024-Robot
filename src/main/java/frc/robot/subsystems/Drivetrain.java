package frc.robot.subsystems;




import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



/**
 * for swerve
 */
public class Drivetrain extends SubsystemBase {

    SwerveModule frontLeft = new SwerveModule(DriveConstants.frontLeftDriveCANId, DriveConstants.frontLeftTurnCANId);
    SwerveModule frontRight = new SwerveModule(DriveConstants.frontRightDriveCANId, DriveConstants.frontRightTurnCANId);
    SwerveModule backLeft = new SwerveModule(DriveConstants.backLeftDriveCANId, DriveConstants.backLeftTurnCANId);
    SwerveModule backRight = new SwerveModule(DriveConstants.backRightDriveCANId, DriveConstants.backRightTurnCANId);

    SwerveDriveOdometry odometry;

    AHRS gyro;

    public Drivetrain() {
        gyro = new AHRS(Port.kMXP);
        odometry = new SwerveDriveOdometry(
                DriveConstants.kDriveKinematics, 
                gyro.getRotation2d(), 
                new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
                }
        );
    
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
                }
        );
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
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
                pose
        );
    }


    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
           fieldRelative 
           ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getAngle()))
           : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );

       setModuleStates(swerveModuleStates);
    }

    public void lockWheels() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void setModuleStates(SwerveModuleState[] states) {

        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxVelocityMetersPerSec);

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

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return gyro.getRate();
    }
    static class DriveConstants {

        public static int frontRightDriveCANId = 0;
        public static int backRightDriveCANId = 2;
        public static int backLeftDriveCANId = 4;
        public static int frontLeftDriveCANId = 6;

        public static int frontRightTurnCANId = 1;
        public static int backRightTurnCANId = 3;
        public static int backLeftTurnCANId = 5;
        public static int frontLeftTurnCANId = 7;


        public static double maxVelocityMetersPerSec = 4.86;

        public static final double kTrackWidth = Units.inchesToMeters(20);
        public static final double kWheelBase = Units.inchesToMeters(20); 
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    }
}
