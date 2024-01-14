package org.sciborgs1155.robot.Shooter;

import static org.sciborgs1155.robot.Shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogBoth;
import org.sciborgs1155.robot.Robot;

public class LauncherBase extends SubsystemBase implements Logged {
  @LogBoth PIDController pid = new PIDController(kp, kd, ki);
  @LogBoth double targetAngle;

  Ultrasonic sensor = new Ultrasonic(9, 10);
  LauncherBaseIO launcher =
      Robot.isReal() ? new RealLauncherBase(pid, sensor) : new SimLauncherBase(pid, sensor);

  @LogBoth
  public double getCurrentAngle() {
    return launcher.getAngleRelativeToHorizontal();
  }

  @Override
  public void periodic() {
    launcher.setVoltageToReachAngle(targetAngle);
    launcher.updateState();
  }

  public Command setAngle(double angle) {
    return runOnce(
            () -> {
              targetAngle = angle;
            })
        .andThen(Commands.idle());
  }
}
