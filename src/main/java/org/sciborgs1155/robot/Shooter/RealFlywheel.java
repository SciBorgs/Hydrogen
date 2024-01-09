package org.sciborgs1155.robot.Shooter;

import javax.management.relation.Relation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class RealFlywheel implements FlywheelIO {
    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }
    @Override
    public void tick() {
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
