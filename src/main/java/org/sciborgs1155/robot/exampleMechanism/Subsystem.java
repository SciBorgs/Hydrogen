// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.exampleMechanism;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.sciborgs1155.lib.failure.Fallible;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.exampleMechanism.exampleSubmechanism.EmptySubmech;
import org.sciborgs1155.robot.exampleMechanism.exampleSubmechanism.RealSubmech;
import org.sciborgs1155.robot.exampleMechanism.exampleSubmechanism.SimSubmech;
import org.sciborgs1155.robot.exampleMechanism.exampleSubmechanism.SubmechIO;

public class Subsystem extends SubsystemBase implements Fallible, AutoCloseable {

  public static Subsystem createEmpty() {
    return new Subsystem(new EmptySubmech());
  }

  public static Subsystem create() {
    return new Subsystem(Robot.isReal() ? new RealSubmech() : new SimSubmech());
  }

  private final SubmechIO mech;

  /** Creates a new ExampleSubsystem. */
  public Subsystem(SubmechIO mech) {
    this.mech = mech;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public List<HardwareFault> getFaults() {
    // TODO Auto-generated method stub
    return mech.getFaults();
  }

  @Override
  public void close() throws Exception {
    mech.close();
    // close all hardware that is AutoClosable. Used for unit tests
  }
}
