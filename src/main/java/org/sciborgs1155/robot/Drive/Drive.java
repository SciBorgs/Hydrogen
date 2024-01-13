package org.sciborgs1155.robot.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import monologue.Logged;
import monologue.Monologue.LogBoth;
import org.sciborgs1155.robot.Robot;

public class Drive extends SubsystemBase implements Logged {

  public PIDController leftPID = new PIDController(10, 5, 0);
  PIDController rightPID = new PIDController(10, 5, 0);

  @LogBoth DriveIO drive = Robot.isReal() ? new RealDrive() : new SimDrive();

  public Command setMotorSpeeds(Supplier<Double> leftSpeed, Supplier<Double> rightSpeed) {
    return run(
        () ->
            drive.setSpeeds(
                leftPID.calculate(drive.getLeftSpeed(), leftSpeed.get()),
                rightPID.calculate(drive.getRightSpeed(), rightSpeed.get())));
  }

  @Override
  public void periodic() {
    drive.periodic();
  }

  public double getLeftSpeed() {
    return drive.getLeftSpeed();
  }
  public double getRightSpeed() {
    return drive.getRightSpeed();
  }

}
