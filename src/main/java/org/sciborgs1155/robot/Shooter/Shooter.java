package org.sciborgs1155.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogBoth;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends SubsystemBase implements Logged {
    private final CANSparkMax motor = new CANSparkMax(ShooterConstants.deviceID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    @LogBoth
    private final PIDController pidController = new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);
    private final FlywheelSim flyWheelSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(ShooterConstants.kVVoltSecondsPerRotation, 1), DCMotor.getNEO(1), 1);
    
    @LogBoth
    private double lastVelocity;

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
    //auto function, siggy told me to
    //random number
    return 1;
  }

  @Override
  public void periodic() {
      lastVelocity = encoder.getVelocity();
  }
}

//inspiration from https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/rapidreactcommandbot/subsystems/Shooter.java