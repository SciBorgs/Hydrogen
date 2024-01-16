// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface CameraIO {
  
  Optional<EstimatedRobotPose> update();
  PhotonCamera fetchCamera();
  PhotonPoseEstimator fetchPoseEstimator();
}