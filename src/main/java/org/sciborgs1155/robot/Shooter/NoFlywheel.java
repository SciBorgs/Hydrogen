package org.sciborgs1155.robot.Shooter;

public class NoFlywheel implements FlywheelIO {
    @Override
    public double getVelocity() {
        return 0;
    }
    @Override
    public void updateSyntax() {
        
    }
}
