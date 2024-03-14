// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {
  PhotonCamera frontCamera;
  PhotonPoseEstimator frontEstimator;
  PhotonCamera backCamera;
  PhotonPoseEstimator backEstimator;

  boolean updateDuringTeleop = true;
  boolean updateDuringAuto = true;
  boolean updateWhenOverThreshold = true;

  /** Creates a new Vision. */
  public Vision() {
    frontCamera = new PhotonCamera("frontCamera");
    frontEstimator =
        new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            frontCamera,
            frontCameraTransform);
    frontEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    backCamera = new PhotonCamera("backCamera");
    backEstimator =
        new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            backCamera,
            backCameraTransform);
    backEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
  }

  @Override
  public void periodic() {
    frontEstimator.setReferencePose(Robot.drivetrain.getPose());
  }

  public boolean withinEstimateBounds(EstimatedRobotPose estimate, Pose2d pose) {
    return Math.abs(estimate.estimatedPose.toPose2d().getTranslation().getDistance(pose.getTranslation())) < poseEstimateDistanceTolerance 
        && Math.abs(estimate.estimatedPose.toPose2d().getRotation().getDegrees() - pose.getRotation().getDegrees()) < poseEstimateRotTolerance;
  }

  public void updateVision() {

    Optional<EstimatedRobotPose> estimate = frontEstimator.update();
    if (estimate.isPresent()) {
      if (withinEstimateBounds(estimate.get(), Robot.drivetrain.getPose())) {
          Robot.drivetrain.updatePoseEstimate(estimate.get());
      }
    }

    backEstimator.setReferencePose(Robot.drivetrain.getPose());
    estimate = backEstimator.update();
    if (estimate.isPresent()) {
      if (withinEstimateBounds(estimate.get(), Robot.drivetrain.getPose())) {
          Robot.drivetrain.updatePoseEstimate(estimate.get());
      }
    }
  }
}
