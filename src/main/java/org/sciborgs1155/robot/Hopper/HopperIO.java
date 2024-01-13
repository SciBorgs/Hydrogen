package org.sciborgs1155.robot.Hopper;

import monologue.Logged;

public interface HopperIO extends Logged {

  public double getSpeed();

  public void setVoltageToReach(double targetSpeed);

  public boolean atTargetSpeed();

  public default void updateState() {}
  ;
}
