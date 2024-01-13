package org.sciborgs1155.robot.Shooter;

public interface FlyWheelIO {

  /**
   * @param launchSpeed target flywheel speed m/s
   */
  public double launchWithSpeed(double launchSpeed);

  public double getAngularSpeed();

  public boolean isAtTargetSpeed();

  public default void updateState() {}
  ;
}
