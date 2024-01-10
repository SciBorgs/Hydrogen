package org.sciborgs1155.robot.Hopper;

import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.Hopper.HopperConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogBoth;
import org.sciborgs1155.robot.Robot;

public class Hopper extends SubsystemBase implements Logged {
  private final CANSparkMax belt = new CANSparkMax(5, MotorType.kBrushless);
  private final PIDController pid = new PIDController(kp, ki, kd);
  private final SimpleMotorFeedforward feedfoward = new SimpleMotorFeedforward(kS, kV, kA);
  private final DCMotorSim simMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
  @LogBoth private double targetSpeed = 0;
  private RelativeEncoder encoder = belt.getEncoder();
  @LogBoth private double voltage;

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      belt.setVoltage(
          pid.calculate(encoder.getVelocity(), targetSpeed) + feedfoward.calculate(targetSpeed));
    } else
      voltage =
          pid.calculate(simMotor.getAngularVelocityRadPerSec() * WHEEL_RADIUS, targetSpeed)
              + feedfoward.calculate(targetSpeed);
    simMotor.setInputVoltage(voltage);
  }

  @Override
  public void simulationPeriodic() {
    simMotor.update(PERIOD);
  }

  @LogBoth
  private boolean atSpeed() {
    return pid.atSetpoint();
  }

  @LogBoth
  private double speed() {
    return Robot.isReal()
        ? encoder.getVelocity()
        : simMotor.getAngularVelocityRadPerSec() * WHEEL_RADIUS;
  }

  public Command forward() {
    return runOnce(
            () -> {
              targetSpeed = MOTOR_MAX_SPEED;
            })
        .andThen(Commands.idle());
  }

  public Command back() {
    return runOnce(
            () -> {
              targetSpeed = -1 * MOTOR_MAX_SPEED;
            })
        .andThen(Commands.idle());
  }

  public Command stop() {
    return runOnce(
        () -> {
          targetSpeed = 0;
        });
  }
}
