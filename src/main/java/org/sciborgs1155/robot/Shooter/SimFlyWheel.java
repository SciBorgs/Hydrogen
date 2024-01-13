package org.sciborgs1155.robot.Shooter;

import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.Shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimFlyWheel implements FlyWheelIO {
  PIDController pid;
  DCMotorSim motor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);

  public SimFlyWheel(PIDController pid) {
    this.pid = pid;
  }

  @Override
  public double getAngularSpeed() {
    return motor.getAngularVelocityRadPerSec();
  }

  @Override
  public double launchWithSpeed(double launchSpeed) {
    double angularSpeed = launchSpeed / FLYWHEEL_RADIUS;
    double voltage = pid.calculate(getAngularSpeed(), angularSpeed);
    motor.setInputVoltage(voltage);
    return voltage;
  }

  @Override
  public boolean isAtTargetSpeed() {
    return pid.atSetpoint();
  }

  @Override
  public void updateState() {
    motor.update(PERIOD);
  }
}
