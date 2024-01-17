package org.sciborgs1155.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.spline.CubicHermiteSpline;

import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import java.util.Set;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealFlywheel implements FlywheelIO {
  public final CANSparkMax motor = new CANSparkMax(deviceID, MotorType.kBrushless);
  public final RelativeEncoder encoder = motor.getEncoder();

  public RealFlywheel() {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kCoast);
    motor.setSmartCurrentLimit(CURRENT_LIMIT);

    encoder.setPositionConversionFactor(POSITION_FACTOR);
    encoder.setVelocityConversionFactor(VELOCITY_FACTOR);

    SparkUtils.configureFrameStrategy(motor, Set.of(Data.POSITION, Data.VELOCITY, Data.VOLTAGE), Set.of(Sensor.DUTY_CYCLE), false);

    FaultLogger.register(motor);
    
    motor.burnFlash();

    encoder.setPosition(0);
  }

  @Override
  public double velocity() {
    return encoder.getVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public double getMotorAppliedOutput() {
    return motor.getAppliedOutput();
  }
  @Override
  public void close() throws Exception {
      motor.close();
  }
}
