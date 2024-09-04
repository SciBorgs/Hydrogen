package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation3d;

/** GyroIO implementation for nonexistent gyro */
public class NoGyro implements GyroIO {
  private final Rotation3d rotation = new Rotation3d();

  @Override
  public void close() throws Exception {}

  @Override
  public double rate() {
    return 0;
  }

  @Override
  public Rotation3d rotation3d() {
    return rotation;
  }

  @Override
  public void reset() {}
}
