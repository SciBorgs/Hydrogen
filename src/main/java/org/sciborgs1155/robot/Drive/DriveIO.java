package org.sciborgs1155.robot.Drive;

import monologue.Logged;

public interface DriveIO extends Logged {
  public void setSpeeds(double leftSpeed, double rightSpeed);

  public double speed();

  public double getLeftSpeed();

  public double getRightSpeed();

  public void periodic();

  public double getX();

  public double getY();
}
