package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import monologue.Logged;

public interface DriveIO extends AutoCloseable, Logged {

  public Pose2d getPose();

  public void resetOdometry(Pose2d pose);

  public Rotation2d getHeading();

  public void close() throws Exception;
}
