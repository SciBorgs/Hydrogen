package org.sciborgs1155.robot.Hopper;

import static org.sciborgs1155.robot.Constants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimMotor implements MotorIO {
  DCMotorSim motor;

  public SimMotor() {
    this.motor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
  }

  @Override
  public void setVoltageTo(double voltage) {
    this.motor.setInputVoltage(voltage);
    this.motor.update(PERIOD.magnitude());
  }

  @Override
  public double getMotorAngularVelocity() {
    return this.motor.getAngularVelocityRadPerSec();
  }

  @Override
  public void close() {}
  ;

  @Override
  public void resetEncoder() {}
  ;
}
