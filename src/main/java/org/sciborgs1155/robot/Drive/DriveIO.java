package org.sciborgs1155.robot.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import monologue.Logged;

public interface DriveIO extends Logged {
  public void setVoltage(double leftSpeed, double rightSpeed);

  public double getLeftSpeed();

  public double getRightSpeed();

  public void updateState();

  public Pose2d getPose();
}
