package org.sciborgs1155.robot.DriveTrain;

import static org.sciborgs1155.robot.DriveTrain.DriveTrainConstants.*;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class RealDriveTrain implements DriveTrainIO {
  PIDController pid;

  // looking from back of robot
  CANSparkMax frontLeft;
  CANSparkMax frontRight;
  CANSparkMax backLeft;
  CANSparkMax backRight;

  public RealDriveTrain(
      CANSparkMax frontLeft, CANSparkMax frontRight, CANSparkMax backLeft, CANSparkMax backRight) {
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

    CANSparkMax[] motors = {
      frontLeft, frontRight, backLeft, backRight,
    };

    double[] currentModuleStates = {
      frontLeft.getEncoder().getVelocity() * RPM_TO_M_PER_S,
      frontLeft.getEncoder().getVelocity() * RPM_TO_M_PER_S,
      frontLeft.getEncoder().getVelocity() * RPM_TO_M_PER_S,
      frontLeft.getEncoder().getVelocity() * RPM_TO_M_PER_S,
    };

    for (int i = 0; i < currentModuleStates.length; i++) {
      motors[i].setVoltage(
          pid.calculate(currentModuleStates[i], targetModuleStates[i].speedMetersPerSecond));
    }
  }
}
