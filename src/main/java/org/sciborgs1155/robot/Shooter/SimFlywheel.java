package org.sciborgs1155.robot.Shooter;

import org.sciborgs1155.robot.Constants;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import monologue.Monologue.LogBoth;

public class SimFlywheel implements FlywheelIO{
    @LogBoth
    public final FlywheelSim flyWheelSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(ShooterConstants.kVVoltSecondsPerRotation, 1), DCMotor.getNEO(1), 1 );
    
    @Override
    public double getVelocity() {
        return flyWheelSim.getAngularVelocityRadPerSec();
    }
    @Override
    public void tick() {
        flyWheelSim.setInputVoltage(getMotorAppliedOutput());
        flyWheelSim.update(Constants.PERIOD);
        System.out.println(getMotorAppliedOutput());
    }
    @Override
    public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }
    @Override
    public double getMotorAppliedOutput(){
        return motor.getAppliedOutput();
    }
}