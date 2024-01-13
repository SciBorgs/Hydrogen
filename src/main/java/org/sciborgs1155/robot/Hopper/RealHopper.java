package org.sciborgs1155.robot.Hopper;

import static org.sciborgs1155.robot.Hopper.HopperConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

public class RealHopper implements HopperIO {
  CANSparkMax motor = new CANSparkMax(5, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();
  PIDController pid;

  public RealHopper(PIDController pid) {
    this.pid = pid;
  }

  @Override
  public double getSpeed() {
    return motor.getEncoder().getVelocity();
  }

  @Override
  public double setVoltageToReach(double targetSpeed) {
    double voltage = pid.calculate(encoder.getVelocity(), targetSpeed);
    motor.setVoltage(pid.calculate(encoder.getVelocity(), targetSpeed));
    return voltage;
  }

  @Override
  public boolean atTargetSpeed() {
    return pid.atSetpoint();
  }

  @Override
  public void updateState() {}
}
