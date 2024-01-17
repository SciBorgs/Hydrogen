package org.sciborgs1155.robot.Drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class DriveConstants {
  static double MAXSPEED = 5.0;
  static Pose2d STARTINGPOSE = new Pose2d(5, 5, Rotation2d.fromRadians(0));
  static DifferentialDrivetrainSim STARTINGDIFFERENTIALDRIVESIM =
      new DifferentialDrivetrainSim(
          DCMotor.getNEO(2),
          9,
          8.5,
          60.0,
          Units.inchesToMeters(3),
          0.7112,
          VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
}
