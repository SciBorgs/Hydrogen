package org.sciborgs1155.robot.Hopper;

import monologue.Logged;

public interface HopperIO extends Logged {

  public double getSpeed();

  public double setVoltageToReach(double targetSpeed);

  public boolean atTargetSpeed();

  public void updateState();
}
