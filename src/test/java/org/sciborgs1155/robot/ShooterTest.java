package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;
import static org.sciborgs1155.robot.Shooter.ShooterConstants.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.Shooter.FlyWheel;
import org.sciborgs1155.robot.Shooter.LauncherBase;

public class ShooterTest {
  FlyWheel flywheel = new FlyWheel();
  LauncherBase launcher = new LauncherBase();
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

  /*@Test
  public void testLauncherBase() {
    run(launcher.setAngle(Math.PI / 4));
    fastForward(500);
    assertEquals(Math.PI/4 , launcher.getCurrentAngle());
    }*/

  // setAngle() relies on WPILib Ultrasonic Sensors, which do not seem to provide a way to set the
  // initial measurement in simulation.

}
