package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;
import static org.sciborgs1155.robot.Hopper.HopperConstants.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.Hopper.Hopper;

public class HopperTest {
  Hopper hopper = new Hopper();
  final double DELTA = 2;

  @BeforeEach
  public void setup() {
    setupHAL();
  }

  @Test
  public void testMovement() {
    run(hopper.forward());
    fastForward(500);
    assertEquals(MOTOR_MAX_SPEED, hopper.speed(), DELTA);
    // reaches correct speed in forward direction

    run(hopper.back());
    fastForward(500);
    assertEquals(MOTOR_MAX_SPEED * -1, hopper.speed(), DELTA);
    // reaches correct speed in reverse direction
  }
}
