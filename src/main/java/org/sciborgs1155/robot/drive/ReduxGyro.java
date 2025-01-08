package org.sciborgs1155.robot.drive;

import static org.sciborgs1155.robot.Ports.Drive.CANANDGYRO;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.geometry.Rotation3d;
import org.sciborgs1155.lib.FaultLogger;

/** GyroIO implementation for NavX */
public class ReduxGyro implements GyroIO {
  private final Canandgyro canandgyro = new Canandgyro(CANANDGYRO);

  public ReduxGyro() {
    FaultLogger.register(canandgyro);

    // See https://docs.reduxrobotics.com/canandgyro/programming/normal-operation#party-mode
    canandgyro.setPartyMode(5);
  }

  @Override
  public double rate() {
    return canandgyro.getAngularVelocityYaw();
  }

  @Override
  public Rotation3d rotation3d() {
    return canandgyro.getRotation3d();
  }

  @Override
  public void reset() {
    canandgyro.setYaw(0);
  }

  @Override
  public void close() throws Exception {}
}
