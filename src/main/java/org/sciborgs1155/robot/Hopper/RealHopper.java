package org.sciborgs1155.robot.Hopper;

import static org.sciborgs1155.robot.Hopper.HopperConstants.*;

public class RealHopper implements HopperIO {
  RealMotor motor = new RealMotor(HOPPER_PORT);

  @Override
  public double getAngularVelocityOfMotor() {
    return motor.getMotorAngularVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltageTo(voltage);
  }
}
