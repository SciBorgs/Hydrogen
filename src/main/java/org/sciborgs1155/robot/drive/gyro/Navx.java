package org.sciborgs1155.robot.drive.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.SPI;

public class Navx implements GyroIO {
  private AHRS ahrs;

  public Navx() {
    ahrs = new AHRS(SPI.Port.kMXP);
  }

  public double getRate() {
    return ahrs.getRate();
  }

  @Override
  public Rotation2d getRotation2d() {
    return ahrs
        .getRotation2d(); // dunno if using the built in method is faster/better, but if not, this
    // can be removed
  }

  @Override
  public Rotation3d getRotation3d() {
    return ahrs
        .getRotation3d(); // dunno if using the built in method is faster/better, but if not, this
    // can be removed
  }

  public double getYaw() {
    return ahrs.getYaw();
  }

  public double getPitch() {
    return ahrs.getPitch();
  }

  public double getRoll() {
    return ahrs.getRoll();
  }

  public void reset() {
    ahrs.reset();
  }

  @Override
  public void close() throws Exception {}
}
