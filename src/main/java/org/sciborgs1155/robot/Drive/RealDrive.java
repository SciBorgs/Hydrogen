package org.sciborgs1155.robot.Drive;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import java.util.Set;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealDrive implements DriveIO {

  CANSparkMax frontLeft = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax rearLeft = new CANSparkMax(2, MotorType.kBrushless);

  CANSparkMax frontRight = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax rearRight = new CANSparkMax(4, MotorType.kBrushless);

  RelativeEncoder leftEncoder = frontLeft.getEncoder();
  RelativeEncoder rightEncoder = frontRight.getEncoder();

  ADIS16448_IMU gyro = new ADIS16448_IMU();

  Pose2d pose = new Pose2d();

  DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(
          Rotation2d.fromDegrees(gyro.getAngle()),
          leftEncoder.getPosition(),
          rightEncoder.getPosition());

  public RealDrive() {

    frontLeft.restoreFactoryDefaults();
    frontRight.restoreFactoryDefaults();
    rearLeft.restoreFactoryDefaults();
    rearRight.restoreFactoryDefaults();

    frontLeft.setSmartCurrentLimit(5);
    frontRight.setSmartCurrentLimit(5);
    rearLeft.setSmartCurrentLimit(5);
    rearRight.setSmartCurrentLimit(5);

    rearLeft.follow(frontLeft);
    rearRight.follow(frontRight);

    frontLeft.setIdleMode(IdleMode.kBrake);
    frontRight.setIdleMode(IdleMode.kBrake);
    rearRight.setIdleMode(IdleMode.kBrake);
    rearLeft.setIdleMode(IdleMode.kBrake);

    frontLeft.setInverted(true);

    SparkUtils.configureFrameStrategy(
        frontLeft, Set.of(Data.VELOCITY), Set.of(Sensor.DUTY_CYCLE), true);
    SparkUtils.configureFrameStrategy(
        frontRight, Set.of(Data.VELOCITY), Set.of(Sensor.DUTY_CYCLE), true);
    SparkUtils.configureFollowerFrameStrategy(rearLeft);
    SparkUtils.configureFollowerFrameStrategy(rearRight);

    frontLeft.burnFlash();
    frontRight.burnFlash();
    rearLeft.burnFlash();
    rearRight.burnFlash();

    FaultLogger.register(frontLeft);
    FaultLogger.register(frontRight);
    FaultLogger.register(rearLeft);
    FaultLogger.register(rearRight);
  }

  @Override
  public void setVoltage(double leftSpeed, double rightSpeed) {
    frontLeft.setVoltage(leftSpeed);
    frontRight.setVoltage(rightSpeed);
  }

  public Pose2d getPose() {
    return pose;
  }

  public double getX() {
    return pose.getX();
  }

  public double getY() {
    return pose.getY();
  }

  public double getLeftSpeed() {
    return leftEncoder.getVelocity();
  }

  public double getRightSpeed() {
    return rightEncoder.getVelocity();
  }

  @Override
  public void updateState() {
    Rotation2d gyroAngle = Rotation2d.fromDegrees(gyro.getGyroAngleX());
    pose = odometry.update(gyroAngle, leftEncoder.getPosition(), rightEncoder.getPosition());
  }
}
