package org.sciborgs1155.robot.Drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import monologue.Monologue.LogBoth;
import org.sciborgs1155.robot.Constants;

public class SimDrive implements DriveIO {

  Pose2d pose = new Pose2d();

  DifferentialDrivetrainSim sim =
      new DifferentialDrivetrainSim(
          DCMotor.getNEO(2),
          9,
          8.5,
          60.0,
          Units.inchesToMeters(3),
          0.7112,
          VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  @LogBoth Field2d field = new Field2d();

  public SimDrive() {
    sim.setPose(new Pose2d(5, 5, Rotation2d.fromRadians(0)));
  }

  @LogBoth
  public double speed() {
    return (sim.getLeftVelocityMetersPerSecond() + sim.getRightVelocityMetersPerSecond()) / 2;
  }

  public double getLeftSpeed() {
    return sim.getLeftVelocityMetersPerSecond();
  }

  public double getRightSpeed() {
    return sim.getRightVelocityMetersPerSecond();
  }

  @Override
  public void setSpeeds(double leftSpeed, double rightSpeed) {
    sim.setInputs(leftSpeed, rightSpeed);
  }

  public void periodic() {
    sim.update(Constants.PERIOD);
    pose = sim.getPose();
    field.setRobotPose(pose);
  }

  @LogBoth
  public double getX() {
    return pose.getX();
  }

  @LogBoth
  public double getY() {
    return pose.getY();
  }
}
