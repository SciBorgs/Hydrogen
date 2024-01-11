package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.List;
import monologue.Logged;
import monologue.Monologue.LogBoth;

/** Generalized SwerveModule with closed loop control */
public interface ModuleIO extends AutoCloseable, Logged {

  /** Returns the current state of the module. */
  @LogBoth
  public SwerveModuleState getState();

  /** Returns the current position of the module. */
  @LogBoth
  public SwerveModulePosition getPosition();

  /** Sets the desired state for the module. */
  public void setDesiredState(SwerveModuleState desiredState);

  /** Returns the desired state for the module. */
  @LogBoth
  public List<SwerveModuleState> getDesiredState();

  public void setDriveVoltage(double driveVolts);

  public void setRotationVoltage(double rotationVolts);

  @LogBoth
  public double getDriveVoltage();

  @LogBoth
  public double getRotationVoltage();

  @LogBoth
  public Rotation2d getHeading();

  /** Zeroes all the drive encoders. */
  public void resetEncoders();
}
