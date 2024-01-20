package org.sciborgs1155.robot.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import static org.sciborgs1155.robot.Constants.*;

public class SimIntake implements IntakeIO {
  DCMotorSim motor;

  public SimIntake() {
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
