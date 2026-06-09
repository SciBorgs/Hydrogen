package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Ports.Drive.GYRO;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N2;
import java.util.Queue;
import org.sciborgs1155.lib.FaultLogger;

/** GyroIO implementation for a Phoenix 2. */
public class PigeonGyro implements GyroIO {
  private final Pigeon2 gyro = new Pigeon2(GYRO);

  private final Queue<Double> position;
  private final Queue<Double> timestamp;

  private double lastAngularVelocity;
  private double alpha;

  public PigeonGyro() {
    FaultLogger.register(gyro);

    position = OdometryThread.getInstance().registerSignal(() -> gyro.getYaw().getValueAsDouble());
    timestamp = OdometryThread.getInstance().makeTimestampQueue();
  }

  @Override
  public double rate() {
    return gyro.getAngularVelocityZWorld().getValueAsDouble(); // device or world
  }

  @Override
  public Rotation3d rotation3d() {
    return gyro.getRotation3d();
  }

  @Override
  public double[][] odometryData() {
    Drive.LOCK.lock();
    try {
      double[][] data = {
        position.stream().mapToDouble((Double d) -> d).toArray(),
        timestamp.stream().mapToDouble((Double d) -> d).toArray()
      };
      position.clear();
      timestamp.clear();
      return data;
    } finally {
      Drive.LOCK.unlock();
    }
  }

  @Override
  public Vector<N2> acceleration() {
    return VecBuilder.fill(
        gyro.getAccelerationX().getValueAsDouble(), gyro.getAccelerationY().getValueAsDouble());

    // again, not sure if device or world
  }

  @Override
  public double alpha() {
    return alpha;
  }

  @Override
  public void reset(Rotation2d heading) {
    gyro.setYaw(heading.getDegrees());
  }

  @Override
  public void close() throws Exception {}

  @Override
  public void periodic() {
    alpha =
        (gyro.getAngularVelocityZWorld().getValueAsDouble() / 360.0 - lastAngularVelocity)
            / PERIOD.in(Seconds);
    lastAngularVelocity = gyro.getAngularVelocityZWorld().getValueAsDouble() / 360.0;
  }
}
