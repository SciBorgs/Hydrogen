package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Ports.Drive.*;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

public class Drive extends SubsystemBase implements Logged, AutoCloseable {

  @Log.NT private final ModuleIO frontLeft;
  @Log.NT private final ModuleIO frontRight;
  @Log.NT private final ModuleIO rearLeft;
  @Log.NT private final ModuleIO rearRight;

  private final List<ModuleIO> modules;

  @Log.NT private final AHRS imu = new AHRS();

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_OFFSET);

  // Odometry and pose estimation
  private final SwerveDrivePoseEstimator odometry;

  @Log.NT private final Field2d field2d = new Field2d();
  private final FieldObject2d[] modules2d;

  // Rate limiting
  private final SlewRateLimiter xLimiter =
      new SlewRateLimiter(MAX_ACCEL.in(MetersPerSecondPerSecond));
  private final SlewRateLimiter yLimiter =
      new SlewRateLimiter(MAX_ACCEL.in(MetersPerSecondPerSecond));

  @Log.NT private double speedMultiplier = 1;

  public static Drive create() {
    return Robot.isReal()
        ? new Drive(
            new MAXSwerveModule(FRONT_LEFT_DRIVE, FRONT_LEFT_TURNING, ANGULAR_OFFSETS.get(0)),
            new MAXSwerveModule(FRONT_RIGHT_DRIVE, FRONT_RIGHT_TURNING, ANGULAR_OFFSETS.get(1)),
            new MAXSwerveModule(REAR_LEFT_DRIVE, REAR_LEFT_TURNING, ANGULAR_OFFSETS.get(2)),
            new MAXSwerveModule(REAR_RIGHT_DRIVE, REAR_RIGHT_TURNING, ANGULAR_OFFSETS.get(3)))
        : new Drive(
            new SparkSimModule(), new SparkSimModule(), new SparkSimModule(), new SparkSimModule());
  }

  public Drive(ModuleIO frontLeft, ModuleIO frontRight, ModuleIO rearLeft, ModuleIO rearRight) {
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.rearLeft = rearLeft;
    this.rearRight = rearRight;

    modules = List.of(frontLeft, frontRight, rearLeft, rearRight);
    modules2d = new FieldObject2d[modules.size()];

    odometry =
        new SwerveDrivePoseEstimator(kinematics, getHeading(), getModulePositions(), new Pose2d());

    for (int i = 0; i < modules2d.length; i++) {
      modules2d[i] = field2d.getObject("module-" + i);
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Returns the heading of the robot, based on our pigeon
   *
   * @return A Rotation2d of our angle
   */
  public Rotation2d getHeading() {
    return Robot.isReal() ? imu.getRotation2d() : Rotation2d.fromRadians(simulatedHeading);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
  }

  /** Deadbands and squares inputs */
  private static double scale(double input) {
    input = MathUtil.applyDeadband(input, Constants.DEADBAND);
    return Math.copySign(input * input, input);
  }

  /** Drives the robot based on a {@link DoubleSupplier} for x y and omega velocities */
  public Command drive(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vOmega) {
    return run(
        () ->
            drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xLimiter.calculate(
                        scale(vx.getAsDouble()) * MAX_SPEED.in(MetersPerSecond) * speedMultiplier),
                    yLimiter.calculate(
                        scale(vy.getAsDouble()) * MAX_SPEED.in(MetersPerSecond) * speedMultiplier),
                    scale(vOmega.getAsDouble())
                        * MAX_ANGULAR_SPEED.in(RadiansPerSecond)
                        * speedMultiplier,
                    getHeading())));
  }

  /**
   * Drives the robot based on profided {@link ChassisSpeeds}.
   *
   * <p>This method uses {@link ChassisSpeeds#discretize(ChassisSpeeds, double)} to reduce skew.
   *
   * @param speeds The desired chassis speeds.
   */
  public void drive(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, Constants.PERIOD.in(Seconds));

    setModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired ModuleIO states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    if (desiredStates.length != modules.size()) {
      throw new IllegalArgumentException("desiredStates must have the same length as modules");
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED.in(MetersPerSecond));

    for (int i = 0; i < modules.size(); i++) {
      modules.get(i).setDesiredState(desiredStates[i]);
    }
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    modules.forEach(ModuleIO::resetEncoders);
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeading() {
    return runOnce(imu::reset);
  }

  /** Returns the pitch of the drive gyro */
  public double getPitch() {
    return imu.getPitch();
  }

  private SwerveModuleState[] getModuleStates() {
    return modules.stream().map(ModuleIO::state).toArray(SwerveModuleState[]::new);
  }

  private SwerveModulePosition[] getModulePositions() {
    return modules.stream().map(ModuleIO::position).toArray(SwerveModulePosition[]::new);
  }

  /** Updates pose estimation based on provided {@link EstimatedRobotPose} */
  public void updateEstimates(EstimatedRobotPose... poses) {
    for (int i = 0; i < poses.length; i++) {
      odometry.addVisionMeasurement(poses[i].estimatedPose.toPose2d(), poses[i].timestampSeconds);
      field2d.getObject("Cam-" + i + " Est Pose").setPose(poses[i].estimatedPose.toPose2d());
    }
  }

  @Override
  public void periodic() {
    odometry.update(getHeading(), getModulePositions());

    field2d.setRobotPose(getPose());

    for (int i = 0; i < modules2d.length; i++) {
      var module = modules.get(i);
      var transform = new Transform2d(MODULE_OFFSET[i], module.position().angle);
      modules2d[i].setPose(getPose().transformBy(transform));
    }
  }

  // jank
  private double simulatedHeading = 0.0;

  @Override
  public void simulationPeriodic() {
    simulatedHeading +=
        kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond
            * Constants.PERIOD.in(Seconds);
  }

  /** Stops drivetrain */
  public Command stop() {
    return runOnce(() -> drive(new ChassisSpeeds()));
  }

  /** Sets the drivetrain to an "X" configuration, preventing movement */
  public Command lock() {
    var front = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    var back = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    return run(() -> setModuleStates(new SwerveModuleState[] {front, back, back, front}));
  }

  /** Sets a new speed multiplier for the robot, this affects max cartesian and angular speeds */
  public Command setSpeedMultiplier(double multiplier) {
    return runOnce(() -> speedMultiplier = multiplier);
  }

  public void close() throws Exception {
    frontLeft.close();
    frontRight.close();
    rearLeft.close();
    rearRight.close();
  }
}
