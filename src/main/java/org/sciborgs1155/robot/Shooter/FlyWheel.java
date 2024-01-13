package org.sciborgs1155.robot.Shooter;

import static org.sciborgs1155.robot.Shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogBoth;
import org.sciborgs1155.robot.Robot;

public class FlyWheel extends SubsystemBase implements Logged {
  @LogBoth final PIDController pid = new PIDController(kp, ki, kd);

  @LogBoth public double targetSpeed = 0;
  public final FlyWheelIO flywheel = Robot.isReal() ? new RealFlyWheel(pid) : new SimFlyWheel(pid);

  @LogBoth
  private double getVoltage() {
    return flywheel.launchWithSpeed(targetSpeed);
  }

  @LogBoth
  public double getShooterSpeed() {
    return flywheel.getAngularSpeed();
  }

  public boolean isAtTargetSpeed() {
    return flywheel.isAtTargetSpeed();
  }

  @Override
  public void periodic() {
    flywheel.launchWithSpeed(targetSpeed);
    flywheel.updateState();
  }

  public Command launch() {
    return runOnce(
            () -> {
              targetSpeed = PROJECTILE_LAUNCH_SPEED;
            })
        .andThen(Commands.idle());
  }

  public Command stop() {
    return runOnce(
            () -> {
              targetSpeed = 0;
            })
        .andThen(Commands.idle());
  }
}
