package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;
import static org.sciborgs1155.robot.Hopper.HopperConstants.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.Hopper.Hopper;

public class HopperTest {
  Hopper hopper;
  final double DELTA = 2;

  @BeforeEach
  public void setup() {
    hopper = new Hopper();
    setupHAL();
  }

  @Test
  public void testMovement() {
    run(hopper.forward());
    fastForward(500);
    assertEquals(HOPPER_ANGULAR_SPEED, hopper.motorAngularVelocity(), DELTA);
    // reaches correct speed in forward direction

    run(hopper.back());
    fastForward(500);
    assertEquals(HOPPER_ANGULAR_SPEED * -1, hopper.motorAngularVelocity(), DELTA);
    // reaches correct speed in reverse direction
  }
}
