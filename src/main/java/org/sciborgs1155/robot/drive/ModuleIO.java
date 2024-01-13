package org.sciborgs1155.robot.drive;

import monologue.Logged;

/** Generalized SwerveModule with closed loop control */
public interface ModuleIO extends AutoCloseable, Logged {
  public void setDriveVoltage(double voltage);

  public void setTurnVoltage(double voltage);

  public double getDrivePosition();

  public double getDriveVelocity();

  public double getTurnPosition();

  public double getTurnVelocity();

  public void resetEncoders();

  @Override
  public void close();
}
