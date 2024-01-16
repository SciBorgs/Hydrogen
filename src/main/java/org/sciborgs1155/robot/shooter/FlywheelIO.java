package org.sciborgs1155.robot.shooter;

import monologue.Logged;
import monologue.Annotations.Log;

public interface FlywheelIO extends Logged, AutoCloseable {
  @Log
  public double velocity();

  public void setVoltage(double voltage);

}

// IO real none sim ideas inspired by Asa and
// https://github.com/SciBorgs/ChargedUp-2023/blob/main/src/main/java/org/sciborgs1155/robot/arm/
