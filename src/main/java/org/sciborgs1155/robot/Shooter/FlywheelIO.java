package org.sciborgs1155.robot.Shooter;

import javax.management.relation.Relation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public interface FlywheelIO {
    double getVelocity();
    void updateSyntax();
}
