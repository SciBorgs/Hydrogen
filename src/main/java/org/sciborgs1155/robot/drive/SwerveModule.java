package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.drive.DriveConstants.SwerveModule.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.SwerveModule.Turning;

/** Class to encapsulate a swerve module controlled by a Talon or Flex controller */
public class SwerveModule implements Logged, AutoCloseable {
  private final ModuleIO module;

  private final PIDController drivePID;
  private final PIDController turnPID;

  private final SimpleMotorFeedforward driveFeedforward;

  private final Rotation2d angularOffset;

  private SwerveModuleState setpoint = new SwerveModuleState();

  /**
   * Constructs a SwerveModule for rev's MAX Swerve using vortexes (flex) or krakens (talon).
   *
   * @param module talon OR flex swerve module
   * @param angularOffset offset from drivetrain
   */
  public SwerveModule(ModuleIO module, double angularOffset) {
    this.module = module;
    drivePID = new PIDController(Driving.PID.P, Driving.PID.I, Driving.PID.D);
    turnPID = new PIDController(Turning.PID.P, Turning.PID.I, Turning.PID.D);

    driveFeedforward = new SimpleMotorFeedforward(Driving.FF.S, Driving.FF.V, Driving.FF.A);

    this.angularOffset = Rotation2d.fromRadians(angularOffset);

    setpoint = new SwerveModuleState();
  }

  public SwerveModuleState state() {
    return new SwerveModuleState(
        module.getDriveVelocity(),
        Rotation2d.fromRadians(module.getTurnPosition()).minus(angularOffset));
  }

  public SwerveModulePosition position() {
    return new SwerveModulePosition(
        module.getDrivePosition(),
        Rotation2d.fromRadians(module.getTurnPosition()).minus(angularOffset));
  }
  ;

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(angularOffset);
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint =
        SwerveModuleState.optimize(
            correctedDesiredState, Rotation2d.fromRadians(module.getTurnPosition()));

    double driveFF = driveFeedforward.calculate(setpoint.speedMetersPerSecond);
    double driveVoltage = driveFF + drivePID.calculate(setpoint.speedMetersPerSecond);
    module.setDriveVoltage(driveVoltage);

    double turnVoltage = turnPID.calculate(setpoint.angle.getRadians());
    module.setTurnVoltage(turnVoltage);
  }

  @Log.NT
  public SwerveModuleState desiredState() {
    return setpoint;
  }

  public void setDriveVoltage(double voltage) {
    module.setDriveVoltage(voltage);
  }

  public void setTurnVoltage(double voltage) {
    module.setTurnVoltage(voltage);
  }

  public void resetEncoders() {
    module.resetEncoders();
  }

  @Override
  public void close() {
    module.close();
  }
}
