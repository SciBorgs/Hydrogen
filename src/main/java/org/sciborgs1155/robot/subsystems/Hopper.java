package org.sciborgs1155.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static org.sciborgs1155.robot.Constants.*;

public class Hopper extends SubsystemBase {
    private final CANSparkMax belt = new CANSparkMax(1, MotorType.kBrushless);

    public Command forward() {
        return runOnce(() -> {belt.setVoltage(HOPPER_VOLTAGE);});
    }
    public Command back() {
        return runOnce(() -> {belt.setVoltage(-1 * HOPPER_VOLTAGE);});
    }
   
    public Command stop() {
        return runOnce(() -> {belt.setVoltage(0);});
    }
  
}


