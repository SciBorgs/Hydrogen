package org.sciborgs1155.robot.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Logged;
import monologue.Annotations.Log;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;
import static org.sciborgs1155.lib.SparkUtils.*;

import org.sciborgs1155.robot.Robot;

public class Shooter extends SubsystemBase implements Logged, AutoCloseable {
  @Log.File private final FlywheelIO flywheel = Robot.isReal() ? new RealFlywheel() : new SimFlywheel();

  @Log.NT
  private final PIDController pidController =
      new PIDController(kp, ki, kd);
      

  private final SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(
          kSVolts, kVVoltSecondsPerRotation);

  @Log.NT
  public boolean isAtGoal() {
    return pidController.atSetpoint();
  }

  public Shooter() {
    setDefaultCommand(runOnce(
      () -> pidController.setSetpoint(0.001)
    ).andThen(Commands.idle(this)));
  }
  
  @Log.NT
  public double getVelocity() {
    return flywheel.velocity();
  }

  public Command shoot() {
    return run(
      () -> pidController.setSetpoint(CONSTANT_TARGET_RPS)
    ); 
  }

  @Override
  public void periodic() {
    flywheel.setVoltage(
        pidController.calculate(flywheel.velocity())
            + feedForward.calculate(pidController.getSetpoint()));
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
