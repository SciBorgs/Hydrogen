package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.drive.DriveConstants.SwerveModule.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.List;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.robot.drive.DriveConstants.SwerveModule.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.SwerveModule.Turning;

public class TalonSwerveModule implements ModuleIO {
  private final TalonFX driveMotor;
  private final CANSparkMax turnMotor;

  private final SparkAbsoluteEncoder turnEncoder;
  private final SparkPIDController turnFeedback;

  private final Rotation2d angularOffset;

  private final PositionVoltage moveRequest = new PositionVoltage(0);

  private SwerveModuleState setpoint = new SwerveModuleState();

  public TalonSwerveModule(int drivePort, int turnPort, Rotation2d angularOffset) {
    driveMotor = new TalonFX(drivePort);
    turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
    this.angularOffset = angularOffset;

    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    turnFeedback = turnMotor.getPIDController();

    configureDrive(driveMotor.getConfigurator());
    configureTurn();

    resetEncoders();

    driveMotor.getPosition().setUpdateFrequency(100);
    driveMotor.getVelocity().setUpdateFrequency(100);
  }

  // resolve magic numbers
  private void configureDrive(TalonFXConfigurator cfg) {
    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    toApply.CurrentLimits.SupplyCurrentLimit = 50;
    toApply.ClosedLoopGeneral.ContinuousWrap = true;
    toApply.Slot0.kP = Driving.PID.P;
    toApply.Slot0.kI = Driving.PID.I;
    toApply.Slot0.kD = Driving.PID.D;
    toApply.Slot0.kS = Driving.FF.S;
    toApply.Slot0.kV = Driving.FF.V;
    toApply.Slot0.kA = Driving.FF.A;
    cfg.apply(toApply);
  }

  private void configureTurn() {
    turnMotor.restoreFactoryDefaults();
    turnMotor.setInverted(false);
    turnMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.setOpenLoopRampRate(0);
    turnMotor.setSmartCurrentLimit(20);

    turnEncoder.setInverted(Turning.ENCODER_INVERTED);

    turnEncoder.setPositionConversionFactor(
        Turning.CONVERSION.in(Turning.CONVERSION.unit().numerator().per(Rotations)));
    turnEncoder.setVelocityConversionFactor(
        Turning.CONVERSION.in(Turning.CONVERSION.unit().numerator().per(Rotations)));

    SparkUtils.enableContinuousPIDInput(
        turnFeedback, 0, Turning.CONVERSION.in(Radians.per(Radians)));

    turnFeedback.setFeedbackDevice(turnEncoder);

    turnEncoder.setInverted(Turning.ENCODER_INVERTED);

    turnFeedback.setP(Turning.PID.P);
    turnFeedback.setI(Turning.PID.I);
    turnFeedback.setD(Turning.PID.D);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMotor.getVelocity().getValueAsDouble(),
        Rotation2d.fromRotations(turnEncoder.getPosition()).minus(angularOffset));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getPosition().getValueAsDouble(),
        Rotation2d.fromRotations(turnEncoder.getPosition()).minus(angularOffset));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;

    // assumed we will have absolute encoders for each of the turn motors
    correctedDesiredState.angle = desiredState.angle.plus(angularOffset);
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint =
        SwerveModuleState.optimize(
            correctedDesiredState, Rotation2d.fromRadians(turnEncoder.getPosition()));

    // TODO update with control that is sure to be correct
    driveMotor.setControl(moveRequest.withSlot(0).withVelocity(setpoint.speedMetersPerSecond));
    turnFeedback.setReference(setpoint.angle.getRadians(), ControlType.kPosition);
  }

  @Override
  public List<SwerveModuleState> getDesiredState() {
    return List.of(setpoint);
  }

  @Override
  public void resetEncoders() {
    driveMotor.setPosition(0);
  }

  @Override
  public void close() throws Exception {
    driveMotor.close();
    turnMotor.close();
  }

  @Override
  public double getDriveVoltage() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveVoltage'");
  }

  @Override
  public double getRotationVoltage() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getRotationVoltage'");
  }

  @Override
  public Rotation2d getHeading() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getHeading'");
  }
}
