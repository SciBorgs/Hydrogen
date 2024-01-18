package org.sciborgs1155.robot.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Drive extends SubsystemBase implements Logged {

  PIDController leftPID = new PIDController(8, 0, 0);
  PIDController rightPID = new PIDController(8, 0, 0);

  @Log.NT Field2d field = new Field2d();

  @Log.File DriveIO drive = Robot.isReal() ? new RealDrive() : new SimDrive();

  public Command setMotorSpeeds(Supplier<Double> leftSpeed, Supplier<Double> rightSpeed) {
    return run(
        () ->
            drive.setVoltage(
                leftPID.calculate(drive.getLeftSpeed(), leftSpeed.get()),
                rightPID.calculate(drive.getRightSpeed(), rightSpeed.get())));
  }

  public double getLeftSpeed() {
    return drive.getLeftSpeed();
  }

  public double getRightSpeed() {
    return drive.getRightSpeed();
  }

  @Override
  public void periodic() {
    drive.updateState();
    field.setRobotPose(drive.getPose());
  }
}
