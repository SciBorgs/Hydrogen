package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;
import static org.sciborgs1155.robot.Shooter.ShooterConstants.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.Shooter.FlyWheel;

public class ShooterTest {
  FlyWheel flywheel = new FlyWheel();
  final double DELTA = 2;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(5000, 0);
  }

  @Test
  public void testMovement() {
    run(flywheel.launch());
    fastForward(500);
    assertEquals(PROJECTILE_LAUNCH_SPEED, flywheel.getShooterSpeed(), DELTA);
  }
}
