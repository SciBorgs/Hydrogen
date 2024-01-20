package org.sciborgs1155.robot.Intake;

import static org.sciborgs1155.robot.Intake.IntakeConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Intake extends SubsystemBase implements Logged {

  @Log.NT final PIDController pid = new PIDController(kp, kd, ki);

  public Intake() {
    pid.setSetpoint(0);
    setDefaultCommand(stop());
  }

  public final IntakeIO intake = Robot.isReal() ? new RealIntake() : new SimIntake();

  @Log.NT
  public double motorAngularVelocity() {
    return intake.getAngularVelocityOfMotor();
  }

  @Override
  public void periodic() {
    intake.setVoltage(pid.calculate(intake.getAngularVelocityOfMotor(), pid.getSetpoint()));
  }

  public Command intake() {
    return runOnce(
        () -> {
          pid.setSetpoint(INTAKE_ANGULAR_SPEED);
        }).andThen(Commands.idle());
  }

  public Command stop() {
    return runOnce(
        () -> {
          pid.setSetpoint(0);
        });
  }

  public Command outtake() {
    return runOnce(
        () -> {
          pid.setSetpoint(-INTAKE_ANGULAR_SPEED);
        }).andThen(Commands.idle());
  }
}
