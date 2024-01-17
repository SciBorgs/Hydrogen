package org.sciborgs1155.robot.Intake;

import static org.sciborgs1155.robot.Intake.IntakeConstants.*;

public class RealIntake implements IntakeIO {
  RealMotor motor = new RealMotor(INTAKE_PORT);

  @Override
  public double getAngularVelocityOfMotor() {
    return motor.getMotorAngularVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltageTo(voltage);
  }
}
