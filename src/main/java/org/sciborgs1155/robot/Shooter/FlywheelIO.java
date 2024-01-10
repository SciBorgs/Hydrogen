package org.sciborgs1155.robot.Shooter;

public interface FlywheelIO {
    default double getVelocity() {return 0;};
    default void tick(){};
    default void setVoltage(double voltage){};
    default double getMotorAppliedOutput() {return 0;};
}

//IO real none sim ideas inspired by Asa and 
//https://github.com/SciBorgs/ChargedUp-2023/blob/main/src/main/java/org/sciborgs1155/robot/arm/