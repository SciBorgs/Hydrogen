package org.sciborgs1155.robot.drive;

import static org.sciborgs1155.robot.Ports.Drive.*;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.List;
import monologue.Annotations.Log;

public class SwerveReal implements SwerveIO {

  @Log.NT private final ModuleIO frontLeft;
  @Log.NT private final ModuleIO frontRight;
  @Log.NT private final ModuleIO rearLeft;
  @Log.NT private final ModuleIO rearRight;
  private final List<ModuleIO> modules;

  private final SwerveDrivePoseEstimator odometry;
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_OFFSET);

  public enum MotorController {
    TALON,
    FLEX;
  }

  public static SwerveReal create(MotorController type) {
    return type == MotorController.FLEX
        ? new SwerveReal(
            new MAXSwerveModule(FRONT_LEFT_DRIVE, FRONT_LEFT_TURNING, ANGULAR_OFFSETS.get(0)),
            new MAXSwerveModule(FRONT_RIGHT_DRIVE, FRONT_RIGHT_TURNING, ANGULAR_OFFSETS.get(1)),
            new MAXSwerveModule(REAR_LEFT_DRIVE, REAR_LEFT_TURNING, ANGULAR_OFFSETS.get(2)),
            new MAXSwerveModule(REAR_RIGHT_DRIVE, REAR_RIGHT_TURNING, ANGULAR_OFFSETS.get(3)))
        : new SwerveReal(
            new TalonSwerveModule(FRONT_LEFT_DRIVE, FRONT_LEFT_TURNING, ANGULAR_OFFSETS.get(0)),
            new TalonSwerveModule(FRONT_RIGHT_DRIVE, FRONT_RIGHT_TURNING, ANGULAR_OFFSETS.get(1)),
            new TalonSwerveModule(REAR_LEFT_DRIVE, REAR_LEFT_TURNING, ANGULAR_OFFSETS.get(2)),
            new TalonSwerveModule(REAR_RIGHT_DRIVE, REAR_RIGHT_TURNING, ANGULAR_OFFSETS.get(3)));
  }

  public SwerveReal(
      ModuleIO frontLeft, ModuleIO frontRight, ModuleIO rearLeft, ModuleIO rearRight) {
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.rearLeft = rearLeft;
    this.rearRight = rearRight;

    modules = List.of(frontLeft, frontRight, rearLeft, rearRight);
    odometry =
        new SwerveDrivePoseEstimator(kinematics, getHeading(), getModulePositions(), new Pose2d());
  }

  @Override
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  @Override
  /**
   * TODO (use navx lib) Returns the heading of the robot, based on our pigeon
   *
   * @return A Rotation2d of our angle
   */
  public Rotation2d getHeading() {
    return new Rotation2d();
  }

  @Override
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
  }

  private SwerveModulePosition[] getModulePositions() {
    return modules.stream().map(ModuleIO::getPosition).toArray(SwerveModulePosition[]::new);
  }

  @Override
  public void close() throws Exception {
    frontLeft.close();
    frontRight.close();
    rearLeft.close();
    rearRight.close();
  }
}
