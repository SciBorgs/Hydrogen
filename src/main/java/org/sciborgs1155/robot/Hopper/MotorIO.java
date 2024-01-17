package org.sciborgs1155.robot.Hopper;

public interface MotorIO {

  public void setVoltageTo(double voltage);

  public void resetEncoder();

  public void close();

  public double getMotorAngularVelocity();
}
