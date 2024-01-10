package org.sciborgs1155.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class RealFlywheel implements FlywheelIO {
    public final CANSparkMax motor = new CANSparkMax(ShooterConstants.deviceID, MotorType.kBrushless);
    public final RelativeEncoder encoder = motor.getEncoder();

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }
    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
    @Override
    public double getMotorAppliedOutput(){
        return motor.getAppliedOutput();
    }
}
