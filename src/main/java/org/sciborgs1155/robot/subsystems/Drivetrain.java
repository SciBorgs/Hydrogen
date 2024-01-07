package org.sciborgs1155.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax motorLeft = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax motorRight = new CANSparkMax(2, MotorType.kBrushless);

  public Drivetrain() {
    setDefaultCommand(stop());
  }

  public Command go(double voltageX, double voltageY) {
    return runOnce(
        () -> {
          motorLeft.setVoltage(voltageY);
          motorRight.setVoltage(voltageX);
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          motorLeft.setVoltage(0);
          motorRight.setVoltage(0);
        });
  }
}
