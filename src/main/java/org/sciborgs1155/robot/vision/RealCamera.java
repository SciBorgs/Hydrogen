// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.vision;
import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.sciborgs1155.robot.vision.CameraIO;
import org.sciborgs1155.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RealCamera implements CameraIO {
  /** Creates a new RealVision. */
  private PhotonCamera camera;
  private PhotonPoseEstimator photonEstimator;


  public RealCamera() {
    camera = new PhotonCamera(null); 
    photonEstimator =
    new PhotonPoseEstimator(
            kTagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            camera,
            kRobotToCam
            
    );
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.

  @Override
  public Optional<EstimatedRobotPose> update(){
    var visionEst = photonEstimator.update();      
    return visionEst; 
  };



}
