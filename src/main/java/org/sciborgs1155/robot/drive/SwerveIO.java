package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import monologue.Annotations.Log;
import monologue.Logged;

/** Generalized SwerveModule with closed loop control */
public interface SwerveIO extends AutoCloseable, Logged {

  @Log.NT
  Pose2d getPose();

  void resetOdometry(Pose2d pose);

  default Rotation2d getHeading() {
    return getRotation().toRotation2d();
  }

  @Log.NT
  SwerveModuleState[] getModuleStates();

  @Log.NT
  SwerveModulePosition[] getModulePositions();

  @Log.NT
  Rotation3d getRotation();

  public void close() throws Exception;
}
