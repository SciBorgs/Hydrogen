package org.sciborgs1155.robot.Intake;

import static org.sciborgs1155.robot.Intake.IntakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogBoth;

public class Intake extends SubsystemBase implements Logged {

  private final CANSparkMax motorWheelRight = new CANSparkMax(1, MotorType.kBrushless);

  @LogBoth
  private double speed() {
    return motorWheelRight.get();
  }

  public Command intake() {
    return runOnce(
        () -> {
          motorWheelRight.set(0.5);
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          motorWheelRight.setVoltage(0);
        });
  }

  public Command outtake() {
    return runOnce(
        () -> {
          motorWheelRight.setVoltage(-1 * INTAKE_VOLTAGE);
        });
  }
}
