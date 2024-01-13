package org.sciborgs1155.robot.Drive;

import org.sciborgs1155.robot.Constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import monologue.Monologue.LogBoth;

public class SimDrive implements DriveIO {

  @LogBoth PIDController leftPID = new PIDController(10, 5, 0);
  @LogBoth PIDController rightPID = new PIDController(10, 5, 0);

  Pose2d pose = new Pose2d();

  DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(DCMotor.getNEO(2),
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

  @Override
  public void setSpeeds(double leftSpeed, double rightSpeed) {
    leftPID.calculate(sim.getLeftVelocityMetersPerSecond(), leftSpeed);
    rightPID.calculate(sim.getRightVelocityMetersPerSecond(), rightSpeed);
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
