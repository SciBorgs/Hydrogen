package org.sciborgs1155.robot.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Logged;
import monologue.Monologue.LogBoth;
import monologue.Monologue.LogFile;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;
import static org.sciborgs1155.lib.SparkUtils.*;

import org.sciborgs1155.robot.Robot;

public class Shooter extends SubsystemBase implements Logged, AutoCloseable {
  @LogFile private final FlywheelIO flywheel = Robot.isReal() ? new RealFlywheel() : new SimFlywheel();

  @LogBoth
  private final PIDController pidController =
      new PIDController(kp, ki, kd);
      

  private final SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(
          kSVolts, kVVoltSecondsPerRotation);

  private double targetRPS;

  @LogBoth
  public boolean isAtGoal() {
    return pidController.atSetpoint();
  }

  public Shooter() {
    //"flywheel should always me moving" - asa
    setDefaultCommand(shoot(() -> 0.000001));
  }
  
  @LogBoth
  public double getVelocity() {
    return flywheel.velocity();
  }

  public Command shoot(DoubleSupplier setpointRPS) {
    // Run the shooter flywheel at the desired setpoint using feedforward and feedback
    return run(() ->
            flywheel.setVoltage(
                pidController.calculate(flywheel.velocity(), setpointRPS.getAsDouble())
                    + feedForward.calculate(setpointRPS.getAsDouble())))
        // Wait until the shooter has reached the setpoint, and then run the feeder
        .withName("Shoot");
  }

  public Command changeTargetRPS(DoubleSupplier change) {
    return runOnce(
      () -> targetRPS += change.getAsDouble()
    );
  }
  @LogBoth
  public double getTargetRPS() {
      return targetRPS;
  }

  @Override
  public void close() throws Exception {
      flywheel.close();
  }
}

// inspiration from
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/rapidreactcommandbot/subsystems/Shooter.java
// IO inspiration by Asa and
// https://github.com/SciBorgs/ChargedUp-2023/blob/main/src/main/java/org/sciborgs1155/robot/arm/
// More IO advice from Siggy
