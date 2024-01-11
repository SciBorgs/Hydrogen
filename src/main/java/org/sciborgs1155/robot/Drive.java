package org.sciborgs1155.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import monologue.Logged;
import monologue.Monologue.LogBoth;

public class Drive extends SubsystemBase implements Logged {
  public static enum Direction {
    FORWARD,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    BACKWARD,
    BACKWARD_LEFT,
    BACKWARD_RIGHT,
    TURN_RIGHT,
    TURN_LEFT
  }
  CANSparkMax frontLeft = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax rearLeft = new CANSparkMax(2, MotorType.kBrushless);

  CANSparkMax frontRight = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax rearRight = new CANSparkMax(4, MotorType.kBrushless);
  DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

  RelativeEncoder leftEncoder = frontLeft.getEncoder();
  RelativeEncoder rightEncoder = frontRight.getEncoder();

  ADIS16448_IMU gyro = new ADIS16448_IMU();

  DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(2), 7.29, 7.5, 60.0, 
    Units.inchesToMeters(3), 0.7112, 
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));


  Pose2d pose = new Pose2d();
  @LogBoth
  Field2d field = new Field2d();
  // Mechanism2d mech2d = new Mechanism2d(0.7, 0.7, new Color8Bit(Color.kAquamarine));
  // MechanismRoot2d root = mech2d.getRoot("drive", 0, 0);

  DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(
          Rotation2d.fromDegrees(gyro.getAngle()),
          leftEncoder.getPosition(),
          rightEncoder.getPosition()
          );
        

  public Drive() {
    frontLeft.setInverted(true); // if you want to invert the entire side you can do so here
    rearLeft.follow(frontLeft);
    rearRight.follow(frontRight);
    sim.setPose(new Pose2d(5, 5, Rotation2d.fromRadians(0)));
  }

  public Command setSpeeds(Supplier<Double> leftSpeed, Supplier<Double> rightSpeed) {
    return Robot.isReal() ? run(() -> drive.tankDrive(leftSpeed.get(), rightSpeed.get()))
     : run(() -> sim.setInputs(5 * leftSpeed.get(), 5 * rightSpeed.get()));
  }

  public Command move(Direction d, double speed) {
    return 
      switch (d) {
        case FORWARD_LEFT ->  setSpeeds(() -> 0.5 * speed, () -> speed);
        case FORWARD_RIGHT ->  setSpeeds(() -> speed, () -> 0.5 * speed);
        case BACKWARD_LEFT ->  setSpeeds(() -> -0.5 * speed, () -> -speed);
        case BACKWARD_RIGHT ->  setSpeeds(() -> -speed, () -> -0.5 * speed);

        case FORWARD ->  setSpeeds(() -> speed, () -> speed);
        case BACKWARD ->  setSpeeds(() -> -speed, () -> -speed);
        case TURN_LEFT ->  setSpeeds(() -> -speed, () -> speed);
        case TURN_RIGHT ->  setSpeeds(() -> speed, () -> -speed);
      };
    }

  @Override
  public void periodic() {
    var gyroAngle = Rotation2d.fromDegrees(gyro.getGyroAngleX());
    if (Robot.isReal()) {
      pose = odometry.update(gyroAngle, leftEncoder.getPosition(), rightEncoder.getPosition()); 
    } else {
      sim.update(Constants.PERIOD);
      pose = sim.getPose();
    }
  }

  @Override
  public void simulationPeriodic() {
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
