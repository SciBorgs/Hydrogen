package org.sciborgs1155.robot.shooter;

public class NoFlywheel implements FlywheelIO {
  @Override
  public double velocity() {
    return 0.0;
  }

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void close() throws Exception {}
}
