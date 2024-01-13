package org.sciborgs1155.robot.Intake;

import static org.sciborgs1155.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimIntake implements IntakeIO {
  PIDController pid;
  DCMotorSim motor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);

  public SimIntake(PIDController pid) {
    this.pid = pid;
  }

  @Override
  public double getIntakeSpeed() {
    return motor.getAngularVelocityRadPerSec();
  }

  @Override
  public void setVoltageToReachSpeed(double targetTranslationalSpeed) {
    motor.setInputVoltage(pid.calculate(getIntakeSpeed(), targetTranslationalSpeed));
  }

  @Override
  public boolean atTargetSpeed() {
    return pid.atSetpoint();
  }

  @Override
  public void updateState() {
    motor.update(PERIOD);
  }
}
