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
  public double getSpeed() {
    return motor.getAngularVelocityRadPerSec() * FLYWHEEL_RADIUS;
  }

  @Override
  public double launchWithSpeed(double launchSpeed) {
    double voltage = pid.calculate(getSpeed(), launchSpeed);
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
