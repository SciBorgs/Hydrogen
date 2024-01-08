package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.List;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.drive.DriveConstants.SwerveModule.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.SwerveModule.Turning;

/** Class to encapsulate a simulated rev max swerve module */
public class SimModule implements ModuleIO {

  private final DCMotorSim drive =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Driving.FF.V, Driving.FF.A),
          DCMotor.getNEO(1),
          Driving.GEARING.in(Radians.per(Radians)));
  private final DCMotorSim turn =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Turning.FF.V, Turning.FF.A),
          DCMotor.getNeo550(1),
          Turning.MOTOR_GEARING.in(Radians.per(Radians)));

  private final PIDController driveFeedback =
      new PIDController(Driving.PID.P * 115, Driving.PID.I, Driving.PID.D * 115.0 / 1000);
  private final PIDController turnFeedback =
      new PIDController(Turning.PID.P * 2, Turning.PID.I, Turning.PID.D * 2.0 / 1000);

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(Driving.FF.S, Driving.FF.V, Driving.FF.A);

  private SwerveModuleState setpoint = new SwerveModuleState();

  public SimModule() {
    turnFeedback.enableContinuousInput(0, Turning.CONVERSION.in(Radians.per(Radians)));
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        drive.getAngularVelocityRadPerSec(),
        Rotation2d.fromRadians(turn.getAngularVelocityRadPerSec()));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        drive.getAngularPositionRad(), Rotation2d.fromRadians(turn.getAngularPositionRad()));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint =
        SwerveModuleState.optimize(
            desiredState, Rotation2d.fromRadians(turn.getAngularPositionRad()));

    final double driveFF = driveFeedforward.calculate(setpoint.speedMetersPerSecond);

    final double driveFB =
        driveFeedback.calculate(drive.getAngularVelocityRadPerSec(), setpoint.speedMetersPerSecond);

    final double turnFB =
        turnFeedback.calculate(turn.getAngularPositionRad(), setpoint.angle.getRadians());

    drive.setInputVoltage(driveFB + driveFF);
    drive.update(Constants.PERIOD);
    turn.setInputVoltage(turnFB);
    turn.update(Constants.PERIOD);
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return setpoint;
  }

  @Override
  public void resetEncoders() {
    drive.setState(VecBuilder.fill(0, 0));
    turn.setState(VecBuilder.fill(0, 0));
  }

  @Override
  public void close() {}

  @Override
  public List<Fault> getFaults() {
    return List.of();
  }
}
