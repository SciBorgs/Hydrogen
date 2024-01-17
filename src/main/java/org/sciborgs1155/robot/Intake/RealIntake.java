package org.sciborgs1155.robot.Intake;

import static org.sciborgs1155.robot.Intake.IntakeConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import org.sciborgs1155.lib.FaultLogger;

public class RealIntake implements IntakeIO {
  CANSparkMax motor = new CANSparkMax(INTAKE_PORT, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();

  public RealIntake() {
    this.motor.restoreFactoryDefaults();
    this.motor.setIdleMode(IdleMode.kCoast);
    this.encoder = this.motor.getEncoder();
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
