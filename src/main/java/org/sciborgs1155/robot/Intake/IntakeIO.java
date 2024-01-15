package org.sciborgs1155.robot.Intake;

import monologue.Logged;

public interface IntakeIO extends Logged {

  public double getSpeed();

  public void setVoltage(double voltage);
}
