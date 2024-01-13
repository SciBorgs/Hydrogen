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
  public final HopperIO hopper = Robot.isReal() ? new RealHopper(pid) : new SimHopper(pid);

  @LogBoth public double targetSpeed = 0;

  @LogBoth
  public double currentSpeed() {
    return hopper.getSpeed();
  }

  @LogBoth private boolean isAtTarget = hopper.atTargetSpeed();

  @Override
  public void periodic() {
    hopper.setVoltageToReach(targetSpeed);
    hopper.updateState();
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
