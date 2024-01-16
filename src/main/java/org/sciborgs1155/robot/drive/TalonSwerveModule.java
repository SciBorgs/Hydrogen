package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class TalonSwerveModule implements ModuleIO {
  private final TalonFX driveMotor;
  private final CANSparkMax turnMotor;

  private final SparkAbsoluteEncoder turnEncoder;

  public TalonSwerveModule(int drivePort, int turnPort) {
    driveMotor = new TalonFX(drivePort);
    turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);

    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

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
    cfg.apply(toApply);
  }

  private void configureTurn() {
    turnMotor.restoreFactoryDefaults();
    turnMotor.setInverted(false);
    turnMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.setOpenLoopRampRate(0);
    turnMotor.setSmartCurrentLimit(20);

    turnEncoder.setInverted(Turning.ENCODER_INVERTED);

    turnEncoder.setPositionConversionFactor(Turning.POSITION_FACTOR.in(Radians));
    turnEncoder.setVelocityConversionFactor(Turning.POSITION_FACTOR.in(Radians));

    turnEncoder.setInverted(Turning.ENCODER_INVERTED);
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
  public double getDrivePosition() {
    return driveMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getDriveVelocity() {
    return driveMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public Rotation2d getRotation() {
    return Rotation2d.fromRadians(turnEncoder.getPosition());
  }

  @Override
  public void resetEncoders() {
    driveMotor.setPosition(0);
  }

  @Override
  public void close() {
    turnMotor.close();
    driveMotor.close();
  }
}
