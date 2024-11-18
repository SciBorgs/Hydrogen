package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class TalonModule implements ModuleIO {
  private final TalonFX driveMotor; // Kraken X60
  private final TalonFX turnMotor; // Kraken X60 -- NEW

  private final StatusSignal<Double> drivePos;
  private final StatusSignal<Double> driveVelocity;
  // private final SparkAbsoluteEncoder turningEncoder;

  private final VelocityVoltage velocityOut = new VelocityVoltage(0);
  private final PositionVoltage radiansOut = new PositionVoltage(0);

  // private final SparkPIDController turnPID;
  private final SimpleMotorFeedforward driveFF;
  private final SimpleMotorFeedforward turnFF;

  private final Rotation2d angularOffset;

  @Log.NT private SwerveModuleState setpoint = new SwerveModuleState();

  private Rotation2d lastRotation;

  private final String name;

  public TalonModule(int drivePort, int turnPort, Rotation2d angularOffset, String name) {
    driveMotor = new TalonFX(drivePort);
    turnMotor = new TalonFX(turnPort);

    drivePos = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveFF =
        new SimpleMotorFeedforward(Driving.FF.TALON.S, Driving.FF.TALON.V, Driving.FF.TALON.A);
    turnFF = new SimpleMotorFeedforward(Turning.FF.S, Turning.FF.V, Turning.FF.A);

    drivePos.setUpdateFrequency(1 / SENSOR_PERIOD.in(Seconds));
    driveVelocity.setUpdateFrequency(1 / SENSOR_PERIOD.in(Seconds));

    TalonFXConfiguration talonDriveConfig = new TalonFXConfiguration();
    TalonFXConfiguration talonTurnConfig = new TalonFXConfiguration();
    // reset config
    driveMotor.getConfigurator().apply(talonDriveConfig);
    turnMotor.getConfigurator().apply(talonTurnConfig);

    talonDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonDriveConfig.Feedback.SensorToMechanismRatio = Driving.POSITION_FACTOR.in(Meters);
    talonDriveConfig.CurrentLimits.SupplyCurrentLimit = Driving.CURRENT_LIMIT.in(Amps);

    talonTurnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonTurnConfig.Feedback.SensorToMechanismRatio = Turning.POSITION_FACTOR.in(Radians);
    talonTurnConfig.CurrentLimits.SupplyCurrentLimit = Turning.CURRENT_LIMIT.in(Amps);

    talonDriveConfig.Slot0.kP = Driving.PID.TALON.P;
    talonDriveConfig.Slot0.kI = Driving.PID.TALON.I;
    talonDriveConfig.Slot0.kD = Driving.PID.TALON.D;

    talonTurnConfig.Slot0.kP = Turning.PID.P;
    talonTurnConfig.Slot0.kI = Turning.PID.I;
    talonTurnConfig.Slot0.kD = Turning.PID.D;

    driveMotor.getConfigurator().apply(talonDriveConfig);
    turnMotor.getConfigurator().apply(talonTurnConfig);

    TalonUtils.addMotor(driveMotor);
    TalonUtils.addMotor(turnMotor);

    resetEncoders();

    this.name = name;
    this.angularOffset = angularOffset;
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnMotor.setVoltage(voltage);
  }

  @Override
  public double drivePosition() {
    return drivePos.getValueAsDouble();
  }

  @Override
  public double driveVelocity() {
    return driveVelocity.getValueAsDouble();
  }

  @Override
  public Rotation2d rotation() {
    lastRotation =
        Rotation2d.fromRadians(turnMotor.getRotorPosition().getValueAsDouble())
            .minus(angularOffset);
    return lastRotation;
  }

  @Override
  public SwerveModuleState state() {
    return new SwerveModuleState(driveVelocity(), rotation());
  }

  @Override
  public SwerveModulePosition position() {
    return new SwerveModulePosition(drivePosition(), rotation());
  }

  @Override
  public SwerveModuleState desiredState() {
    return setpoint;
  }

  @Override
  public void resetEncoders() {
    driveMotor.setPosition(0);
  }

  @Override
  public void setDriveSetpoint(double velocity) {
    driveMotor.setControl(
        velocityOut.withVelocity(velocity).withFeedForward(driveFF.calculate(velocity)));
  }

  @Override
  public void setTurnSetpoint(double angle) {
    turnMotor.setControl(radiansOut.withPosition(angle).withFeedForward(turnFF.calculate(angle)));
  }

  @Override
  public void updateSetpoint(SwerveModuleState setpoint, ControlMode mode) {
    setpoint = SwerveModuleState.optimize(setpoint, rotation());
    // Scale setpoint by cos of turning error to reduce tread wear
    setpoint.speedMetersPerSecond *= setpoint.angle.minus(rotation()).getCos();

    if (mode == ControlMode.OPEN_LOOP_VELOCITY) {
      setDriveVoltage(driveFF.calculate(setpoint.speedMetersPerSecond));
    } else {
      setDriveSetpoint(setpoint.speedMetersPerSecond);
    }

    setTurnSetpoint(setpoint.angle.getRadians());
    this.setpoint = setpoint;
  }

  @Override
  public void updateInputs(Rotation2d angle, double voltage) {
    setpoint.angle = angle;
    setDriveVoltage(voltage);
    setTurnSetpoint(angle.getRadians());
  }

  @Override
  public void close() {
    turnMotor.close();
    driveMotor.close();
  }
}
