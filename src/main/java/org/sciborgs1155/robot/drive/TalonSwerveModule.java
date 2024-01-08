package org.sciborgs1155.robot.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class TalonSwerveModule implements ModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;

  private SwerveModuleState setpoint = new SwerveModuleState();
  ;

  public TalonSwerveModule(int drivePort, int turnPort) {
    driveMotor = new TalonFX(drivePort);
    turnMotor = new TalonFX(turnPort);

    configureDrive(driveMotor.getConfigurator());
    configureTurn(turnMotor.getConfigurator());

    resetEncoders();

    driveMotor.getPosition().setUpdateFrequency(100);
    turnMotor.getPosition().setUpdateFrequency(100);

    driveMotor.getVelocity().setUpdateFrequency(100);
    turnMotor.getVelocity().setUpdateFrequency(100);
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
  public void setDesiredState(SwerveModuleState desiredState) {}

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
