package org.sciborgs1155.robot.shooter;

import monologue.Logged;
import monologue.Monologue.LogBoth;

public interface FlywheelIO extends Logged {
  @LogBoth
  double getVelocity();

  void setVoltage(double voltage);
}

// IO real none sim ideas inspired by Asa and
// https://github.com/SciBorgs/ChargedUp-2023/blob/main/src/main/java/org/sciborgs1155/robot/arm/
