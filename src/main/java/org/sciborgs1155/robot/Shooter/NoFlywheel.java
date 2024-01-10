package org.sciborgs1155.robot.Shooter;

public class NoFlywheel implements FlywheelIO {
    @Override
    public double getVelocity() {
        return 0.0;
    }
    //maybe possible future use for tick and setVoltage, but currently deleted
    @Override
    public double getMotorAppliedOutput(){
        return 0.0;
    }
}
