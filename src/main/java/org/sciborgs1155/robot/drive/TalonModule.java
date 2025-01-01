package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.robot.drive.DriveConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Set;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.robot.drive.DriveConstants.ControlMode;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class TalonModule implements ModuleIO {
  private final TalonFX driveMotor; // Kraken X60
  private final SparkMax turnMotor; // NEO 550
  private final SparkMaxConfig turnMotorConfig;

  private final StatusSignal<Angle> drivePos;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final SparkAbsoluteEncoder turningEncoder;

  private final VelocityVoltage velocityOut = new VelocityVoltage(0);

  private final SparkClosedLoopController turnPID;
  private final SimpleMotorFeedforward driveFF;

  private final Rotation2d angularOffset;

  @Log.NT private SwerveModuleState setpoint = new SwerveModuleState();

  private Rotation2d lastRotation;

  private final String name;

  public TalonModule(int drivePort, int turnPort, Rotation2d angularOffset, String name) {

    // Drive Motor

    driveMotor = new TalonFX(drivePort);
    drivePos = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveFF =
        new SimpleMotorFeedforward(Driving.FF.TALON.S, Driving.FF.TALON.V, Driving.FF.TALON.A);

    drivePos.setUpdateFrequency(1 / SENSOR_PERIOD.in(Seconds));
    driveVelocity.setUpdateFrequency(1 / SENSOR_PERIOD.in(Seconds));

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    // reset config
    driveMotor.getConfigurator().apply(talonConfig);

    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.Feedback.SensorToMechanismRatio = Driving.POSITION_FACTOR.in(Meters);
    talonConfig.CurrentLimits.SupplyCurrentLimit = Driving.CURRENT_LIMIT.in(Amps);

    talonConfig.Slot0.kP = Driving.PID.TALON.P;
    talonConfig.Slot0.kI = Driving.PID.TALON.I;
    talonConfig.Slot0.kD = Driving.PID.TALON.D;

    driveMotor.getConfigurator().apply(talonConfig);

    TalonUtils.addMotor(driveMotor);

    // Turn Motor

    turnMotor = new SparkMax(turnPort, MotorType.kBrushless);
    turningEncoder = turnMotor.getAbsoluteEncoder();
    turnPID = turnMotor.getClosedLoopController();
    turnMotorConfig = new SparkMaxConfig();

    check(
        turnMotor,
        turnMotor.configure(
            turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

    turnMotorConfig.apply(
        turnMotorConfig
            .closedLoop
            .pid(Turning.PID.P, Turning.PID.I, Turning.PID.D)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI, Math.PI)
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder));

    turnMotorConfig.apply(
        turnMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) Turning.CURRENT_LIMIT.in(Amps)));

    turnMotorConfig.apply(turnMotorConfig.encoder.inverted(true));

    turnMotorConfig.apply(
        turnMotorConfig
            .encoder
            .positionConversionFactor(Turning.POSITION_FACTOR.in(Radians))
            .velocityConversionFactor(Turning.VELOCITY_FACTOR.in(RadiansPerSecond))
            .uvwAverageDepth(2));

    turnMotorConfig.apply(
        SparkUtils.getSignalsConfigurationFrameStrategy(
            Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
            Set.of(Sensor.ABSOLUTE),
            false));

    check(
        turnMotor,
        turnMotor.configure(
            turnMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

    register(turnMotor);

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
        SparkUtils.wrapCall(
                turnMotor,
                Rotation2d.fromRadians(turningEncoder.getPosition()).minus(angularOffset))
            .orElse(lastRotation);
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
    turnPID.setReference(angle, ControlType.kPosition);
  }

  @Override
  public void updateSetpoint(SwerveModuleState setpoint, ControlMode mode) {
    setpoint.optimize(rotation());
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
