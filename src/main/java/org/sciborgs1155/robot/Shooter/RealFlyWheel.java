package org.sciborgs1155.robot.Shooter;

import static org.sciborgs1155.robot.Shooter.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

public class RealFlyWheel implements FlyWheelIO {
  PIDController pid;

  public RealFlyWheel(PIDController pid) {
    this.pid = pid;
  }

  CANSparkMax motor = new CANSparkMax(6, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();

  @Override
  public double getAngularSpeed() {
    return encoder.getVelocity() * RPM_TO_RAD_PER_S;
  }

  @Override
  public double launchWithSpeed(double launchSpeed) {
    double angularSpeed = launchSpeed / FLYWHEEL_RADIUS;
    double voltage = pid.calculate(getAngularSpeed(), angularSpeed);
    motor.setVoltage(voltage);
    return voltage;
  }

  @Override
  public boolean isAtTargetSpeed() {
    return pid.atSetpoint();
  }
}
