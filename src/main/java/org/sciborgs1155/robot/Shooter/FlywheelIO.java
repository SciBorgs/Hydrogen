package org.sciborgs1155.robot.Shooter;

import javax.management.relation.Relation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public interface FlywheelIO {
    double getVelocity();
    void updateSyntax();
}

//IO real none sim ideas inspired by
//https://github.com/SciBorgs/ChargedUp-2023/blob/main/src/main/java/org/sciborgs1155/robot/arm/