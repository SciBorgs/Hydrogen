package org.sciborgs1155.robot.exampleMechanism.exampleSubmechanism;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.lib.failure.Fallible;

public interface SubmechIO extends Fallible, Sendable, AutoCloseable {

  @Override
  default void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub

  }
}
