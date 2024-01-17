package org.sciborgs1155.robot.Intake;

import static org.sciborgs1155.robot.Constants.*;

public class SimIntake implements IntakeIO {
  SimMotor motor = new SimMotor();

  @Override
  public double getAngularVelocityOfMotor() {
    return motor.getMotorAngularVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltageTo(voltage);
  }
}
