package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.List;
import monologue.Annotations.Log;
import monologue.Logged;

/** Generalized SwerveModule with closed loop control */
public interface ModuleIO extends AutoCloseable, Logged {

  /** Returns the current state of the module. */
  @Log.NT
  public SwerveModuleState getState();

  /** Returns the current position of the module. */
  @Log.NT
  public SwerveModulePosition getPosition();

  /** Sets the desired state for the module. */
  public void setDesiredState(SwerveModuleState desiredState);

  /** Returns the desired state for the module. */
  @Log.NT
  public List<SwerveModuleState> getDesiredState();

  @Log.NT
  public double getDriveVoltage();

  @Log.NT
  public double getRotationVoltage();

  @Log.NT
  public Rotation2d getHeading();

  /** Zeroes all the drive encoders. */
  public void resetEncoders();
}
