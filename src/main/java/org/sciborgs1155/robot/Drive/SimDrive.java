package org.sciborgs1155.robot.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import monologue.Annotations.Log;
import org.sciborgs1155.robot.Constants;

public class SimDrive implements DriveIO {

  Pose2d pose = new Pose2d();

  DifferentialDrivetrainSim sim = DriveConstants.STARTINGDIFFERENTIALDRIVESIM;

  @Log.NT Field2d field = new Field2d();

  public SimDrive() {
    sim.setPose(DriveConstants.STARTINGPOSE);
  }

  public double getLeftSpeed() {
    return sim.getLeftVelocityMetersPerSecond();
  }

  public double getRightSpeed() {
    return sim.getRightVelocityMetersPerSecond();
  }

  @Override
  public void setVoltage(double leftSpeed, double rightSpeed) {
    sim.setInputs(leftSpeed, rightSpeed);
  }

  @Override
  public void updateState() {
    sim.update(Constants.PERIOD);
    pose = sim.getPose();
    field.setRobotPose(pose);
  }

  @Override
  public Pose2d getPose() {
    return pose;
  }
}
