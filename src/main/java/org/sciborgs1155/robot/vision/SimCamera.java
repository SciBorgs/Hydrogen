// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.sciborgs1155.robot.Constants;

public class SimCamera implements CameraIO {
    private final PhotonPoseEstimator photonEstimator;
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;
    private PhotonCamera camera;

    public SimCamera(){
      camera = new PhotonCamera(null); 

      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(Constants.TAG_LAYOUT);//Sets up tag layout for simulation

      //Setting up PhotonPoseEstimator
      photonEstimator =
            new PhotonPoseEstimator(
                    Constants.TAG_LAYOUT, 
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                    camera, Constants.ROBOT_TO_CAM);

      photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      //Base sim cam properties to be configured later
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      cameraSim = new PhotonCameraSim(camera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(cameraSim, Constants.ROBOT_TO_CAM);

      cameraSim.enableDrawWireframe(true);
 
    }
    @Override
    public Optional<EstimatedRobotPose> update(){
      var visionEst = photonEstimator.update();      
      return visionEst; 
    };


}
