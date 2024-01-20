package org.sciborgs1155.robot.Hopper;

import static org.sciborgs1155.robot.Hopper.HopperConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Hopper extends SubsystemBase implements Logged {

  @Log.NT final PIDController pid = new PIDController(kp, ki, kd);

  public Hopper() {
    this.pid.setSetpoint(0);
    setDefaultCommand(stop());
  }

  public final HopperIO hopper = Robot.isReal() ? new RealHopper() : new SimHopper();

  @Log.NT
  public double motorAngularVelocity() {
    return hopper.getAngularVelocityOfMotor();
  }

  @Log.NT
  private boolean isAtTarget() {
    return pid.atSetpoint();
  }

  @Override
  public void periodic() {
    hopper.setVoltage(pid.calculate(hopper.getAngularVelocityOfMotor(), pid.getSetpoint()));
  }

  public Command forward() {
    return runOnce(
            () -> {
              pid.setSetpoint(HOPPER_ANGULAR_SPEED);
            })
        .andThen(Commands.idle());
  }

  public Command back() {
    return runOnce(
            () -> {
              pid.setSetpoint(-HOPPER_ANGULAR_SPEED);
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
