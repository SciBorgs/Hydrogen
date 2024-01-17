package org.sciborgs1155.robot.Intake;

import static org.sciborgs1155.lib.SparkUtils.Data.*;
import static org.sciborgs1155.lib.SparkUtils.Sensor.*;
import static org.sciborgs1155.robot.Intake.IntakeConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import java.util.Set;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.SparkUtils;

public class RealIntake implements IntakeIO {
  CANSparkMax motor = new CANSparkMax(INTAKE_PORT, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder();

  public RealIntake() {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kCoast);

    SparkUtils.configureFrameStrategy(motor, Set.of(VELOCITY), Set.of(DUTY_CYCLE), false);

    motor.burnFlash();

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
