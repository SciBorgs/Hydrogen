package org.sciborgs1155.robot.DriveTrain;

public interface DriveTrainIO {
  public void setVoltageToReachVelocity(
      double velocityForward, double velocitySideways, double rotationalVelocity);
}
