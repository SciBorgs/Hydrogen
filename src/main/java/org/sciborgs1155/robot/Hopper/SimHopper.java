package org.sciborgs1155.robot.Hopper;

import static org.sciborgs1155.robot.Constants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimHopper implements HopperIO {
  DCMotorSim motor;

  public SimHopper() {
    motor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
  }

  @Override
  public double getAngularVelocityOfMotor() {
    return motor.getAngularVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setInputVoltage(voltage);
    motor.update(PERIOD);
  }
}
