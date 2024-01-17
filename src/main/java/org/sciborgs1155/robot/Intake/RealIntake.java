package org.sciborgs1155.robot.Intake;

import static org.sciborgs1155.robot.Intake.IntakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class RealIntake implements IntakeIO {
  CANSparkMax motor = new CANSparkMax(7, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();

  @Override
  public double getAngularVelocityOfMotor() {
    return encoder.getVelocity() * RPM_TO_RAD_PER_S;
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
}
