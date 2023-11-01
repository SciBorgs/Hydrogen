// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.sciborgs1155.robot.exampleMechanism;

import static org.sciborgs1155.robot.exampleMechanism.ExampleConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.List;
import org.sciborgs1155.lib.failure.Fallible;
import org.sciborgs1155.lib.failure.HardwareFault;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.exampleMechanism.exampleSubmechanism.*;

public class ExampleSubsystem extends SubsystemBase implements Loggable, Fallible, AutoCloseable {

  public static ExampleSubsystem createNone() {
    return new ExampleSubsystem(new NoSubmech());
  }

  public static ExampleSubsystem create() {
    return new ExampleSubsystem(Robot.isReal() ? new RealSubmech() : new SimSubmech());
  }

  @Log private final SubmechIO submech;

  /** Creates a new ExampleSubsystem. */
  private ExampleSubsystem(SubmechIO submech) {
    this.submech = submech;
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
    return submech.condition();
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
    return submech.getFaults();
  }

  @Override
  public void close() throws Exception {
    submech.close();
    // close all hardware that is AutoClosable. Used for unit tests
  }
}
