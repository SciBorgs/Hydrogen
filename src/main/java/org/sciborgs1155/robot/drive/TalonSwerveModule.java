package org.sciborgs1155.robot.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.sciborgs1155.robot.drive.DriveConstants.SwerveModule.Driving;

public class TalonSwerveModule implements ModuleIO {
  private final TalonFX driveMotor;

  // change to sparkmax (neo 550)
  private final TalonFX turnMotor;

  private final SimpleMotorFeedforward turnFeedForward =
      new SimpleMotorFeedforward(Driving.FF.S, Driving.FF.V, Driving.FF.A);
  private final SimpleMotorFeedforward driveFeedForward =
      new SimpleMotorFeedforward(Driving.FF.S, Driving.FF.V, Driving.FF.A);

  private SwerveModuleState setpoint = new SwerveModuleState();

  public TalonSwerveModule(int drivePort, int turnPort) {
    driveMotor = new TalonFX(drivePort);
    turnMotor = new TalonFX(turnPort);

    driveMotor.getConfigurator().apply(new TalonFXConfiguration());

    configureDrive(driveMotor.getConfigurator());
    configureTurn(turnMotor.getConfigurator());

    resetEncoders();

    driveMotor.getPosition().setUpdateFrequency(100);

    driveMotor.getVelocity().setUpdateFrequency(100);
  }

  // resolve magic numbers
  private void configureDrive(TalonFXConfigurator cfg) {
    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    toApply.CurrentLimits.SupplyCurrentLimit = 50;
    cfg.apply(toApply);
  }

  private void configureTurn(TalonFXConfigurator cfg) {
    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    toApply.CurrentLimits.SupplyCurrentLimit = 20;
    cfg.apply(toApply);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMotor.getVelocity().getValueAsDouble(),
        Rotation2d.fromRotations(turnMotor.getPosition().getValueAsDouble()));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getPosition().getValueAsDouble(),
        Rotation2d.fromRotations(turnMotor.getPosition().getValue()));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(angularOffset);
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint =
        SwerveModuleState.optimize(
            correctedDesiredState, Rotation2d.fromRadians(turnMotor.getPosition().getValue()));

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
    driveMotor.setPosition(0);
    turnMotor.setPosition(0);
  }

  @Override
  public void close() throws Exception {
    driveMotor.close();
    turnMotor.close();
  }
}
