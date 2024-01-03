package org.sciborgs1155.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(ShooterConstants.deviceID, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();
    private final PIDController pidController = new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);

    public Shooter() {
        setDefaultCommand(
        runOnce(
                () -> {
                  motor.disable();
                  //m_feederMotor.disable();
                })
            .andThen(run(() -> {}))
            .withName("Idle"));
  }

  public Command shootCommand(double setpointRPS) {
            // Run the shooter flywheel at the desired setpoint using feedforward and feedback
            run(
                () ->
                    motor.set(
                            pidController.calculate(
                                encoder.getRate(), setpointRPS)))
            // Wait until the shooter has reached the setpoint, and then run the feeder
        .withName("Shoot");
  }
}

