package org.sciborgs1155.robot.Shooter;

import javax.management.relation.Relation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RealFlywheel implements FlywheelIO {
    private final CANSparkMax motor = new CANSparkMax(ShooterConstants.deviceID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }
    @Override
    public void updateSyntax() {
    }
}
