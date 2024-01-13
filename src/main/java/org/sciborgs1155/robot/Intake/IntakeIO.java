package org.sciborgs1155.robot.Intake;

import monologue.Logged;

public interface IntakeIO extends Logged {

  public double getIntakeSpeed();

  public void setVoltageToReachSpeed(double targetTranslationalSpeed);

  public boolean atTargetSpeed();

  public default void updateState() {}
  ;
}
