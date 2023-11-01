package org.sciborgs1155.robot.exampleMechanism.exampleSubmechanism;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.lib.failure.Fallible;

public interface SubmechIO extends Fallible, Sendable, AutoCloseable {

  public boolean condition();

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("condition", this::condition, null);
  }
}
