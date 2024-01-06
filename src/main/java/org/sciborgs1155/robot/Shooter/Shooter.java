package org.sciborgs1155.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(ShooterConstants.deviceID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private final PIDController pidController = new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);

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
            return run(
                () ->
                    motor.set(
                            pidController.calculate(encoder.getVelocity(), setpointRPS)
                                + feedForward.calculate(setpointRPS)))
            // Wait until the shooter has reached the setpoint, and then run the feeder
        .withName("Shoot");
  }

  public double getSpeed(double distance) {
    return 
  }
}

