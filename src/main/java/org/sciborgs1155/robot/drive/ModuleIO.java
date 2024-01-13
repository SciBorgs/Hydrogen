package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import monologue.Annotations.Log;
import monologue.Logged;

/** Generalized SwerveModule with closed loop control */
public interface ModuleIO extends AutoCloseable, Logged {
  /** Returns the current state of the module. */
  @Log.NT
  public SwerveModuleState state();

  /** Returns the current position of the module. */
  @Log.NT
  public SwerveModulePosition position();

  /** Sets the desired state for the module. */
  public void setDesiredState(SwerveModuleState desiredState);

  /** Returns the desired state for the module. */
  @Log.NT
  public SwerveModuleState desiredState();

  /** Zeroes all the drive encoders. */
  public void resetEncoders();
}
