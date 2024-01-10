package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.robot.drive.DriveConstants.SwerveModule.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.SwerveModule.Turning;

/** Class to encapsulate a rev max swerve module */
public class MAXSwerveModule implements ModuleIO {

  private final CANSparkBase driveMotor; // Regular Neo
  private final CANSparkBase turnMotor; // Neo 550

  private final RelativeEncoder driveEncoder;
  private final SparkAbsoluteEncoder turningEncoder;

  private final SparkPIDController driveFeedback;
  private final SparkPIDController turnFeedback;

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(Driving.FF.S, Driving.FF.V, Driving.FF.A);

  private final Rotation2d angularOffset;

  private SwerveModuleState setpoint = new SwerveModuleState();
  private CANSparkBase sparkBase;

  /**
   * Constructs a SwerveModule for rev's MAX Swerve.
   *
   * @param drivePort drive motor port
   * @param turnPort turning motor port
   * @param angularOffset offset from drivetrain
   */
  public MAXSwerveModule(int drivePort, int turnPort, Measure<Angle> angularOffset) {
    driveMotor = SparkUtils.create(drivePort);
    driveMotor.setInverted(false);
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setOpenLoopRampRate(0);
    driveMotor.setSmartCurrentLimit(50);

    turnMotor = SparkUtils.create(turnPort);
    turnMotor.setInverted(false);
    turnMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.setOpenLoopRampRate(0);
    turnMotor.setSmartCurrentLimit(20);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    driveFeedback = driveMotor.getPIDController();
    turnFeedback = turnMotor.getPIDController();

    driveFeedback.setFeedbackDevice(driveEncoder);
    turnFeedback.setFeedbackDevice(turningEncoder);

    turningEncoder.setInverted(Turning.ENCODER_INVERTED);

    driveFeedback.setP(Driving.PID.P);
    driveFeedback.setI(Driving.PID.I);
    driveFeedback.setD(Driving.PID.D);

    turnFeedback.setP(Turning.PID.P);
    turnFeedback.setI(Turning.PID.I);
    turnFeedback.setD(Turning.PID.D);

    SparkUtils.setConversion(driveEncoder, Driving.CONVERSION);
    SparkUtils.setConversion(turningEncoder, Turning.CONVERSION);

    SparkUtils.enableContinuousPIDInput(
        turnFeedback, 0, Turning.CONVERSION.in(Radians.per(Radians)));

    SparkUtils.disableFrames(driveMotor, 4, 5, 6);
    SparkUtils.disableFrames(turnMotor, 4, 6);

    driveEncoder.setPosition(0);
    this.angularOffset = Rotation2d.fromRadians(angularOffset.in(Radians));

    sparkBase.getPIDController();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(),
        Rotation2d.fromRadians(turningEncoder.getPosition()).minus(angularOffset));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        Rotation2d.fromRadians(turningEncoder.getPosition()).minus(angularOffset));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(angularOffset);
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint =
        SwerveModuleState.optimize(
            correctedDesiredState, Rotation2d.fromRadians(turningEncoder.getPosition()));

    double driveFF = driveFeedforward.calculate(setpoint.speedMetersPerSecond);
    driveFeedback.setReference(setpoint.speedMetersPerSecond, ControlType.kCurrent, 0, driveFF);
    turnFeedback.setReference(setpoint.angle.getRadians(), ControlType.kPosition);
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return setpoint;
  }

  @Override
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  @Override
  public void close() {
    driveMotor.close();
    turnMotor.close();
  }
}
