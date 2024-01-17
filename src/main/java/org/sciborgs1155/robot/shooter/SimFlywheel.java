package org.sciborgs1155.robot.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import monologue.Annotations.Log;
import org.sciborgs1155.robot.Constants;

public class SimFlywheel implements FlywheelIO {
  @Log.NT
  public final FlywheelSim flyWheelSim =
      new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(ShooterConstants.kVVoltSecondsPerRotation, 1),
          DCMotor.getNEO(1),
          1);

  @Override
  public double velocity() {
    return flyWheelSim.getAngularVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double voltage) {
    flyWheelSim.setInputVoltage(voltage);
    flyWheelSim.update(Constants.PERIOD);
  }
  @Override
  public void close() throws Exception {}
}
