package org.sciborgs1155.robot.Hopper;

import static org.sciborgs1155.robot.Hopper.HopperConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class RealHopper implements HopperIO {
  CANSparkMax motor = new CANSparkMax(5, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();

  @Override
  public double getAngularVelocityOfMotor() {
    return motor.getEncoder().getVelocity() * RPM_TO_RAD_PER_S;
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
}
