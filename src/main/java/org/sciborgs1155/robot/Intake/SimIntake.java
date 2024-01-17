package org.sciborgs1155.robot.Intake;

import static org.sciborgs1155.robot.Constants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimIntake implements IntakeIO {
  DCMotorSim motor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);

  @Override
  public double getAngularVelocityOfMotor() {
    return motor.getAngularVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setInputVoltage(voltage);
    motor.update(PERIOD.magnitude());
  }
}
