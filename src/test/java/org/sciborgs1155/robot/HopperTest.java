package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;
import static org.sciborgs1155.robot.Hopper.HopperConstants.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.Hopper.Hopper;

public class HopperTest {
  Hopper hopper = new Hopper();

  @BeforeEach
  public void setup() {
    assert HAL.initialize(5000, 0);
  }

  @Test
  public void testMovementForward() {
    run(hopper.forward());
    assertEquals(MOTOR_MAX_SPEED, hopper.targetSpeed);
  }

  @Test
  public void testMovementBackwards() {
    run(hopper.back());
    assertEquals(MOTOR_MAX_SPEED * -1, hopper.targetSpeed);
  }

  @Test
  public void reachesSetpointWhenMovingForward() {
    run(hopper.forward());
    fastForward(500);
    assertEquals(MOTOR_MAX_SPEED, Math.round(hopper.currentSpeed()));
  }

  @Test
  public void reachesSetpointWhenMovingBack() {
    run(hopper.back());
    fastForward(500);
    assertEquals(MOTOR_MAX_SPEED * -1, Math.round(hopper.currentSpeed()));
  }
}
