package org.sciborgs1155.robot.Drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  static final double MAXSPEED = 5.0;
  static final Pose2d STARTINGPOSE = new Pose2d(5, 5, Rotation2d.fromRadians(0));

  static final DCMotor DRIVEMOTOR =  DCMotor.getNEO(2);
  static final double GEARING = 9;
  static final double JKGMETERSSQUARED = 8.5;
  static final double MASSKG = 60.0;
  static final double WHEELRADIUSMETERS = Units.inchesToMeters(3);
  static final double TRACKWIDTHMETERS = 0.7112;
  static final Matrix<N7,N1> MEASUREMENTSSTDDEVS = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);
}