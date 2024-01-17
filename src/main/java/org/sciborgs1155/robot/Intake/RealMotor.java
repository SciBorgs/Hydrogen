package org.sciborgs1155.robot.Intake;

import static org.sciborgs1155.robot.Hopper.HopperConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class RealMotor implements MotorIO {
  CANSparkMax motor;
  RelativeEncoder encoder;

  public RealMotor(int hopperMotorPort) {
    this.motor = new CANSparkMax(hopperMotorPort, MotorType.kBrushless);
    this.motor.restoreFactoryDefaults();
    this.motor.setIdleMode(IdleMode.kCoast);
    this.encoder = this.motor.getEncoder();
  }

  @Override
  public void setVoltageTo(double voltage) {
    this.motor.setVoltage(voltage);
  }

  @Override
  public void resetEncoder() {
    this.encoder.setPosition(0);
  }

  @Override
  public double getMotorAngularVelocity() {
    return this.encoder.getVelocity() * RPM_TO_RAD_PER_S;
  }

  @Override
  public void close() {
    this.motor.close();
  }
}
