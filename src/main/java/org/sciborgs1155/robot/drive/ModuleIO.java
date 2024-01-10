package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
  public SwerveModuleState getDesiredState();

  /** Zeroes all the drive encoders. */
  public void resetEncoders();
}
