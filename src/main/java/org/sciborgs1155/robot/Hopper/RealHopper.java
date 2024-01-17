package org.sciborgs1155.robot.Hopper;

import static org.sciborgs1155.robot.Hopper.HopperConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import org.sciborgs1155.lib.FaultLogger;

public class RealHopper implements HopperIO {
  CANSparkMax motor = new CANSparkMax(HOPPER_PORT, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();

  public RealHopper() {
    this.motor.restoreFactoryDefaults();
    this.motor.setIdleMode(IdleMode.kCoast);
    FaultLogger.register(motor);
  }

  @Override
  public double getAngularVelocityOfMotor() {
    return encoder.getVelocity() * RPM_TO_RAD_PER_S;
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
}
