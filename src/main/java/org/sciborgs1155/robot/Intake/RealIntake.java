package org.sciborgs1155.robot.Intake;

import static org.sciborgs1155.robot.Intake.IntakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

public class RealIntake implements IntakeIO {
  CANSparkMax motor = new CANSparkMax(7, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();
  PIDController pid;

  public RealIntake(PIDController pid) {
    this.pid = pid;
  }

  @Override
  public double getIntakeSpeed() {
    return encoder.getVelocity() * MOTOR_RADIUS;
  }

  @Override
  public void setVoltageToReachSpeed(double targetTranslationalSpeed) {
    motor.setVoltage(pid.calculate(getIntakeSpeed(), targetTranslationalSpeed));
  }

  @Override
  public boolean atTargetSpeed() {
    return pid.atSetpoint();
  }
}
