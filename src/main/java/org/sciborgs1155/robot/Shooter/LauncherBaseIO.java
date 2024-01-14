package org.sciborgs1155.robot.Shooter;

public interface LauncherBaseIO {
  public double getAngleRelativeToHorizontal();

  public double setVoltageToReachAngle(double targetAngle);

  public boolean atTargetAngle();

  public default void updateState() {}
  ;
}
