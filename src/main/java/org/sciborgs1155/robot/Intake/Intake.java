package org.sciborgs1155.robot.Intake;

import static org.sciborgs1155.robot.Intake.IntakeConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogBoth;
import org.sciborgs1155.robot.Robot;

public class Intake extends SubsystemBase implements Logged {

  final PIDController pid = new PIDController(kp, kd, ki);
  public final IntakeIO intake = Robot.isReal() ? new RealIntake(pid) : new SimIntake(pid);
  @LogBoth public double targetSpeed = 0;

  @LogBoth
  public double speed() {
    return intake.getIntakeSpeed();
  }

  @Override
  public void periodic() {
    intake.setVoltageToReachSpeed(targetSpeed);
    intake.updateState();
  }

  public Command intake() {
    return runOnce(
        () -> {
          targetSpeed = INTAKE_SPEED;
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          targetSpeed = 0;
        });
  }

  public Command outtake() {
    return runOnce(
        () -> {
          targetSpeed = -1 * INTAKE_SPEED;
        });
  }
}
