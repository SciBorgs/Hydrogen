package org.sciborgs1155.robot.Hopper;

public class SimHopper implements HopperIO {

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
