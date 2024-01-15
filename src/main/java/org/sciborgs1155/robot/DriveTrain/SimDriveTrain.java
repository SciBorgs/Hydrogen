package org.sciborgs1155.robot.DriveTrain;

import static org.sciborgs1155.robot.DriveTrain.DriveTrainConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimDriveTrain implements DriveTrainIO {
  PIDController pid;

  // looking from back of robot
  DCMotorSim frontLeft;
  DCMotorSim frontRight;
  DCMotorSim backLeft;
  DCMotorSim backRight;

  public SimDriveTrain(
      DCMotorSim frontLeft, DCMotorSim frontRight, DCMotorSim backLeft, DCMotorSim backRight) {
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;
  }

  /*"Positive x values represent moving toward
  the front of the robot whereas positive y values represent moving toward the left of the robot."
   */
  double distanceTofrontFromCenter = ROBOT_LENGTH / 2;
  double distanceToSideFromCenter = ROBOT_WIDTH / 2;

  Translation2d locationOfFrontLeft = new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
  Translation2d locationOfFrontRight = new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2);
  Translation2d locationOfBackLeft = new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
  Translation2d locationOfBackRight = new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2);
  SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          locationOfFrontLeft, locationOfFrontRight, locationOfBackLeft, locationOfBackRight);

  @Override
  public void setVoltageToReachVelocity(
      double velocityForward, double velocitySideways, double rotationalVelocity) {
    ChassisSpeeds targetState =
        new ChassisSpeeds(velocityForward, velocitySideways, rotationalVelocity);
    SwerveModuleState[] targetModuleStates = kinematics.toSwerveModuleStates(targetState);

    DCMotorSim[] motors = {
      frontLeft, frontRight, backLeft, backRight,
    };

    double[] currentModuleStates = {
      frontLeft.getAngularVelocityRadPerSec(),
      frontLeft.getAngularVelocityRadPerSec(),
      frontLeft.getAngularVelocityRadPerSec(),
      frontLeft.getAngularVelocityRadPerSec(),
    };

    for (int i = 0; i < currentModuleStates.length; i++) {
      motors[i].setInput(
          pid.calculate(currentModuleStates[i], targetModuleStates[i].speedMetersPerSecond));
    }
  }
}
