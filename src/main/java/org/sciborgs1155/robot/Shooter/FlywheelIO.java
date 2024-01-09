package org.sciborgs1155.robot.Shooter;

// import javax.management.relation.Relation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public interface FlywheelIO {
    public final CANSparkMax motor = new CANSparkMax(ShooterConstants.deviceID, MotorType.kBrushless);
    public final RelativeEncoder encoder = motor.getEncoder();
    public FlywheelSim flyWheelSim = null;
    double getVelocity();
    void tick();
    void setVoltage(double voltage);
    double getMotorAppliedOutput();
}

//IO real none sim ideas inspired by Asa and 
//https://github.com/SciBorgs/ChargedUp-2023/blob/main/src/main/java/org/sciborgs1155/robot/arm/