package org.sciborgs1155.robot.Hopper;

import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.Hopper.HopperConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimHopper implements HopperIO {
  PIDController pid;
  DCMotorSim motor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);

  public SimHopper(PIDController pid) {
    this.pid = pid;
  }

  @Override
  public boolean atTargetSpeed() {
    return pid.atSetpoint();
  }

  @Override
  public double getSpeed() {
    return motor.getAngularVelocityRadPerSec() * WHEEL_RADIUS;
  }

  @Override
  public double setVoltageToReach(double targetSpeed) {
    double voltage = pid.calculate(getSpeed(), targetSpeed);
    motor.setInputVoltage(voltage);
    return voltage;
  }

  @Override
  public void updateState() {
    motor.update(PERIOD);
  }
}
