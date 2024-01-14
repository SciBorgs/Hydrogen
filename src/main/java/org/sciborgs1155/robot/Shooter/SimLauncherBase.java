package org.sciborgs1155.robot.Shooter;

import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.Shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimLauncherBase implements LauncherBaseIO {
  DCMotorSim motor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
  Ultrasonic heightSensor;
  PIDController pid;

  public SimLauncherBase(PIDController pid, Ultrasonic heightSensor) {
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
    motor.setInputVoltage(voltage);
    return voltage;
  }

  @Override
  public boolean atTargetAngle() {
    return pid.atSetpoint();
  }

  @Override
  public void updateState() {
    motor.update(PERIOD);
  }
}
