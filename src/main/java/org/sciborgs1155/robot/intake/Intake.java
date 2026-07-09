package org.sciborgs1155.robot.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;

import org.sciborgs1155.robot.Ports;

public class Intake extends SubsystemBase implements AutoCloseable  {
    private final TalonFX RollerIntake;

    public Intake() {
        RollerIntake = new TalonFX(Ports.Intake.ROLLER);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.CURRENT_LIMIT.in(Amps);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        RollerIntake.getConfigurator().apply(config);
    }

    public static Command spin() {
        return Commands.run(() -> , Intake)
    }

    public static Command stop() {
        return Commands.run()
    }

    @Override
    public void close() throws Exception {
        stop();
    }
}
