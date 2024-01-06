package org.sciborgs1155.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static org.sciborgs1155.robot.Constants.*;

public class Intake extends SubsystemBase {   

    private final CANSparkMax motorWheelRight = new CANSparkMax(1, MotorType.kBrushless);;
    private final CANSparkMax motorWheelLeft = new CANSparkMax(2, MotorType.kBrushless);
    
    public Intake() {
        setDefaultCommand(stop());
    }
    public Command intake() {
        return runOnce(() -> {
        motorWheelRight.setVoltage(INTAKE_VOLTAGE);
        motorWheelLeft.setVoltage(INTAKE_VOLTAGE);
        });
    }

    public Command stop() {
        return runOnce(() -> {
        motorWheelRight.setVoltage(0);
        motorWheelLeft.setVoltage(0);});

    }

    public Command outtake() {
        return runOnce(() -> {
        motorWheelRight.setVoltage(-1*INTAKE_VOLTAGE);
        motorWheelLeft.setVoltage(-1*INTAKE_VOLTAGE);
        });
    }

    

}