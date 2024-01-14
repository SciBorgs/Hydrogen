package org.sciborgs1155.robot.Shooter;

import static org.sciborgs1155.robot.Shooter.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Ultrasonic;

public class RealLauncherBase implements LauncherBaseIO {
  CANSparkMax motor = new CANSparkMax(8, MotorType.kBrushless);
  Ultrasonic heightSensor;
  PIDController pid;

  public RealLauncherBase(PIDController pid, Ultrasonic heightSensor) {
    this.pid = pid;
    this.heightSensor = heightSensor; // beneath launcher base (does not move as angle changes)
  }

  @Override
  public double getAngleRelativeToHorizontal() {
    double heightOfLauncher = heightSensor.getRangeMM() / 1000;
    return Math.atan((heightOfLauncher / DISTANCE_FROM_SENSOR_TO_FIXED_BASE_EDGE));
  }

  @Override
  public double setVoltageToReachAngle(double targetAngle) {
    double targetHeight = DISTANCE_FROM_SENSOR_TO_FIXED_BASE_EDGE * Math.atan(targetAngle);
    double currentHeight = heightSensor.getRangeMM() / 1000;
    double voltage = pid.calculate(currentHeight, targetHeight);
    motor.setVoltage(voltage);
    return voltage;
  }

  @Override
  public boolean atTargetAngle() {
    return pid.atSetpoint();
  }
}
