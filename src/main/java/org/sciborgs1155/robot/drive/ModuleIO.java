package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import monologue.Logged;

/** Generalized SwerveModule with closed loop control */
public interface ModuleIO extends AutoCloseable, Logged {
  public void setDriveVoltage(double voltage);

  public void setTurnVoltage(double voltage);

  public double getDrivePosition();

  public double getDriveVelocity();

  public Rotation2d getRotation();

  public double getTurnVelocity();

  public double getDriveVoltage();

  public double getTurnVoltage();

  public void resetEncoders();

  @Override
  public void close();
}
