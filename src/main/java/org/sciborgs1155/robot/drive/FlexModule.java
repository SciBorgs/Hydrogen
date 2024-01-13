package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import org.sciborgs1155.robot.drive.DriveConstants.SwerveModule.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.SwerveModule.Turning;

public class FlexModule implements ModuleIO {

  private final CANSparkBase driveMotor; // Neo Vortex
  private final CANSparkBase turnMotor; // Neo 550

  private final RelativeEncoder driveEncoder;
  private final SparkAbsoluteEncoder turningEncoder;

  /**
   * Constructs a SwerveModule for rev's MAX Swerve.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   */
  public FlexModule(int drivePort, int turnPort) {
    driveMotor = new CANSparkFlex(drivePort, MotorType.kBrushless);
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(false);
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setSmartCurrentLimit(50);

    turnMotor = new CANSparkFlex(turnPort, MotorType.kBrushless);
    turnMotor.restoreFactoryDefaults();
    turnMotor.setInverted(false);
    turnMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.setSmartCurrentLimit(20);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    turningEncoder.setInverted(Turning.ENCODER_INVERTED);

    driveEncoder.setPositionConversionFactor(Driving.CONVERSION.in(Rotations));
    driveEncoder.setVelocityConversionFactor(Driving.CONVERSION.per(Second).in(RPM));

    turningEncoder.setPositionConversionFactor(Turning.CONVERSION.in(Rotations));
    turningEncoder.setVelocityConversionFactor(Turning.CONVERSION.per(Second).in(RPM));

    driveMotor.burnFlash();
    turnMotor.burnFlash();

    resetEncoders();
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
    return driveEncoder.getPosition();
  }

  @Override
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  @Override
  public double getTurnPosition() {
    return turningEncoder.getPosition();
  }

  @Override
  public double getTurnVelocity() {
    return turningEncoder.getVelocity();
  }

  @Override
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  @Override
  public void close() {
    driveMotor.close();
    turnMotor.close();
  }
}
