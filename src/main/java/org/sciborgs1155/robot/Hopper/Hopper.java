package org.sciborgs1155.robot.Hopper;

import static org.sciborgs1155.robot.Hopper.HopperConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogBoth;
import org.sciborgs1155.robot.Robot;

public class Hopper extends SubsystemBase implements Logged {

  final PIDController pid = new PIDController(kp, ki, kd);
  public final HopperIO hopper = Robot.isReal() ? new RealHopper() : new SimHopper();

  @LogBoth public double targetSpeed = pid.getSetpoint();

  @LogBoth
  public double speed() {
    return hopper.getSpeed();
  }

  @LogBoth private boolean isAtTarget = pid.atSetpoint();

  @Override
  public void periodic() {
    hopper.setVoltage(pid.calculate(hopper.getSpeed(), targetSpeed));
  }

  public Command forward() {
    return runOnce(
            () -> {
              pid.setSetpoint(MOTOR_MAX_SPEED);
            })
        .andThen(Commands.idle());
  }

  public Command back() {
    return runOnce(
            () -> {
              pid.setSetpoint(-MOTOR_MAX_SPEED);
            })
        .andThen(Commands.idle());
  }

  public Command stop() {
    return runOnce(
        () -> {
          pid.setSetpoint(0);
        });
  }
}
