package org.sciborgs1155.robot.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Logged;
import monologue.Monologue.LogBoth;
import org.sciborgs1155.robot.Robot;

public class Shooter extends SubsystemBase implements Logged, AutoCloseable {
  private final FlywheelIO flywheel;

  @LogBoth
  private final PIDController pidController =
      new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
      

  private final SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(
          ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);

  @LogBoth public double target;

  @LogBoth
  public boolean isAtGoal() {
    return flywheel.velocity() == target;
  }

  public Shooter(FlywheelIO flywheelIOtype) {
    flywheel =
        flywheelIOtype; // in theory, this sets a final flywheel to the correct type of object thing
    // (like real/fake/no flywheels)
    setDefaultCommand(shoot(() -> 0.1));
  }

  public static Shooter createFromConfigure() {
    // see if you are real
    return Robot.isReal() ? new Shooter(new RealFlywheel()) : new Shooter(new SimFlywheel());
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

  @Override
  public void periodic() {
      SmartDashboard.putNumber("flywheel velocity", flywheel.velocity());
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
