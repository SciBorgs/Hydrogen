package org.sciborgs1155.robot.Drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADIS16448_IMU;

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
    frontLeft.setInverted(true);
    rearLeft.follow(frontLeft);
    rearRight.follow(frontRight);
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
